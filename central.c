// BLE advertisement template
//
// Advertises device name: EE149

#include <stdbool.h>
#include <stdint.h>
#include <stdio.h>
#include <math.h>
#include "nrf.h"
#include "app_util.h"
#include "nrf_twi_mngr.h"
#include "nrf_log.h"
#include "nrf_log_ctrl.h"
#include "nrf_log_default_backends.h"
#include "nrfx_saadc.h"
#include "nrfx_gpiote.h"
#include "nrf_serial.h"
#include "nrf_gpio.h"
#include "nrf_pwr_mgmt.h"
#include "app_error.h"

#include "buckler.h"
#include "simple_ble.h"

#include "max44009.h"
#include "states.h"

#define X_CHANNEL 0
#define Y_CHANNEL 1
#define Z_CHANNEL 2

states state = ACTIVE;

// Create a timer
APP_TIMER_DEF(adv_timer);

// I2C manager
NRF_TWI_MNGR_DEF(twi_mngr_instance, 5, 0);

// Intervals for advertising and connections
static simple_ble_config_t ble_config = {
        // c0:98:e5:49:xx:xx
        .platform_id       = 0x49,    // used as 4th octect in device BLE address
        .device_id         = 0x0003,  // TODO: replace with your lab bench number
        .adv_name          = "MINTY", // Note that this name is not displayed to save room in the advertisement for data.
        .adv_interval      = MSEC_TO_UNITS(1000, UNIT_0_625_MS),
        .min_conn_interval = MSEC_TO_UNITS(500, UNIT_1_25_MS),
        .max_conn_interval = MSEC_TO_UNITS(1000, UNIT_1_25_MS),
};

static simple_ble_service_t led_service = {{
    .uuid128 = {0x70,0x6C,0x98,0x41,0xCE,0x43,0x14,0xA9,
                0xB5,0x4D,0x22,0x2B,0x89,0x10,0xE6,0x32}
}};

static simple_ble_char_t led_state_char = {.uuid16 = 0x1090};
static bool led_state = true;

//acceleration sampling
float supply_volt_ratio = 2.87 / 3;

void saadc_callback (nrfx_saadc_evt_t const * p_event) {
  // don't care about adc callbacks
}

float convert_adc_to_g(nrf_saadc_value_t adc) {
    float result = 0;
    float lsb = (0.6 / (1.0 / 6)) / (2 << 11); 
    float volts = (float)adc * lsb;
    result = (volts - 1.5 * supply_volt_ratio) / (0.42 * supply_volt_ratio);
    return result;
}

float convert_g_to_tilt(char which, float g_x, float g_y, float g_z) {
    float tilt = 0;
    if (which == 'x') {
      tilt = atan(g_x/ sqrt(pow(g_y, 2) + pow(g_z, 2)));
    } else if (which == 'y') {
      tilt = atan(g_y/ sqrt(pow(g_x, 2) + pow(g_z, 2)));
    } else {
      tilt = atan(sqrt(pow(g_x, 2) + pow(g_y, 2)) / g_x);
    }
    return tilt * (180 / 3.14);
 }

nrf_saadc_value_t sample_value (uint8_t channel) {
  nrf_saadc_value_t val;
  ret_code_t error_code = nrfx_saadc_sample_convert(channel, &val);
  APP_ERROR_CHECK(error_code);
  return val;
}

int sensor_reading_updated = 0;

void read_sensor_callback() {
    sensor_reading_updated = 1;
    printf("Updating sensor reading!\n");
    // TODO: implement this function!
    // Use Simple BLE function to read light sensor and put data in
    // advertisement, but be careful about doing it in the callback itself!
}

void ble_evt_write(ble_evt_t const* p_ble_evt) {
    if (simple_ble_is_char_event(p_ble_evt, &left_state_char)) {
      printf("Got write to LEFT characteristic!\n");
      if (led_state) {
        printf("Turning on LEFT!\n");
        nrf_gpio_pin_clear(BUCKLER_LED0);
      } else {
        printf("Turning off LEFT!\n");
        nrf_gpio_pin_set(BUCKLER_LED0);
      }
    }
}

void ble_evt_write(ble_evt_t const* p_ble_evt) {
    if (simple_ble_is_char_event(p_ble_evt, &right_state_char)) {
      printf("Got write to RIGHT characteristic!\n");
      if (led_state) {
        printf("Turning on RIGHT!\n");
        nrf_gpio_pin_clear(BUCKLER_LED0);
      } else {
        printf("Turning off RIGHT!\n");
        nrf_gpio_pin_set(BUCKLER_LED0);
      }
    }
}

void ble_evt_write(ble_evt_t const* p_ble_evt) {
    if (simple_ble_is_char_event(p_ble_evt, &led_state_char)) {
      printf("Got write to LED characteristic!\n");
      if (led_state) {
        printf("Turning on LED!\n");
        nrf_gpio_pin_clear(BUCKLER_LED0);
      } else {
        printf("Turning off LED!\n");
        nrf_gpio_pin_set(BUCKLER_LED0);
      }
    }
}

simple_ble_app_t* simple_ble_app;

int main(void) {
  ret_code_t error_code = NRF_SUCCESS;

  // Initialize

  // initialize RTT library
  error_code = NRF_LOG_INIT(NULL);
  APP_ERROR_CHECK(error_code);
  NRF_LOG_DEFAULT_BACKENDS_INIT();
  printf("Log initialized\n");

  // initialize analog to digital converter
  nrfx_saadc_config_t saadc_config = NRFX_SAADC_DEFAULT_CONFIG;
  saadc_config.resolution = NRF_SAADC_RESOLUTION_12BIT;
  error_code = nrfx_saadc_init(&saadc_config, saadc_callback);
  APP_ERROR_CHECK(error_code);

  // initialize analog inputs
  // configure with 0 as input pin for now
  nrf_saadc_channel_config_t channel_config = NRFX_SAADC_DEFAULT_CHANNEL_CONFIG_SE(0);
  channel_config.gain = NRF_SAADC_GAIN1_6; // input gain of 1/6 Volts/Volt, multiply incoming signal by (1/6)
  channel_config.reference = NRF_SAADC_REFERENCE_INTERNAL; // 0.6 Volt reference, input after gain can be 0 to 0.6 Volts
 
  // specify input pin and initialize that ADC channel
  channel_config.pin_p = BUCKLER_ANALOG_ACCEL_X;
  error_code = nrfx_saadc_channel_init(X_CHANNEL, &channel_config);
  APP_ERROR_CHECK(error_code);

  // specify input pin and initialize that ADC channel
  channel_config.pin_p = BUCKLER_ANALOG_ACCEL_Y;
  error_code = nrfx_saadc_channel_init(Y_CHANNEL, &channel_config);
  APP_ERROR_CHECK(error_code);

  // specify input pin and initialize that ADC channel
  channel_config.pin_p = BUCKLER_ANALOG_ACCEL_Z;
  error_code = nrfx_saadc_channel_init(Z_CHANNEL, &channel_config);
  APP_ERROR_CHECK(error_code);

  // initialize i2c master (two wire interface)
  nrf_drv_twi_config_t i2c_config = NRF_DRV_TWI_DEFAULT_CONFIG;
  i2c_config.scl = BUCKLER_SENSORS_SCL;
  i2c_config.sda = BUCKLER_SENSORS_SDA;
  i2c_config.frequency = NRF_TWIM_FREQ_100K;
  error_code = nrf_twi_mngr_init(&twi_mngr_instance, &i2c_config);
  APP_ERROR_CHECK(error_code);

  // initialize MAX44009 driver
  const max44009_config_t config = {
    .continuous = 0,
    .manual = 0,
    .cdr = 0,
    .int_time = 3,
  };
  max44009_init(&twi_mngr_instance, BUCKLER_LIGHT_INTERRUPT);
  max44009_config(config);
  printf("MAX44009 initialized\n");

  // Setup BLE
  simple_ble_app = simple_ble_init(&ble_config);
  simple_ble_add_service(&led_service);

  simple_ble_add_characteristic(1, 1, 0, 0,
      sizeof(led_state), (uint8_t*)&led_state,
      &led_service, &led_state_char);

  // Start Advertising
  simple_ble_adv_only_name();
  

  // TODO replace this with advertisement sending light data
  const size_t BUFSIZE = 12;
  char accel_buf[BUFSIZE];
  // simple_ble_adv_manuf_data(adv_buf, 3 * sizeof(float));

  // Set a timer to read the light sensor and update advertisement data every second.
  app_timer_init();
  app_timer_create(&adv_timer, APP_TIMER_MODE_REPEATED, (app_timer_timeout_handler_t) read_sensor_callback);
  app_timer_start(adv_timer, APP_TIMER_TICKS(1000), NULL); // 1000 milliseconds
  
  nrf_gpio_cfg_output(BUCKLER_LED0);

  

  while(1) {
    if (state == ACTIVE) {
    if (sensor_reading_updated) {
      nrf_saadc_value_t x_val = sample_value(X_CHANNEL);
      nrf_saadc_value_t y_val = sample_value(Y_CHANNEL);
      nrf_saadc_value_t z_val = sample_value(Z_CHANNEL);  
      float x_accel =  convert_adc_to_g(x_val);
      float y_accel =  convert_adc_to_g(x_val);
      float z_accel =  convert_adc_to_g(x_val);
      printf("X_acceleration: %f", x_accel);
      accel_buf[0] = x_accel;
      accel_buf[4] = y_accel;
      accel_buf[8] = z_accel;
      sensor_reading_updated = 0;
    } else {
      power_manage();
    } 
    } else if (state == LOW_POWER) {
      power_manage();
    }
  }
}
