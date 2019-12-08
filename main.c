// BLE Service Template
//
// Creates a service for changing LED state over BLE

#include <stdbool.h>
#include <stdint.h>
#include "nrf.h"
#include "app_util.h"
#include "nrf_twi_mngr.h"
#include "nrf_gpio.h"
#include "display.h"
#include "nrf_delay.h"
#include "states.h"

#include "simple_ble.h"
#include "buckler.h"

#include "max44009.h"
#include "mpu9250.h"

// I2C manager
NRF_TWI_MNGR_DEF(twi_mngr_instance, 5, 0);
// Enable SoftDevice (used to get RTC running)
nrf_sdh_enable_request();




// Intervals for advertising and connections
static simple_ble_config_t ble_config = {
        // c0:98:e5:49:xx:xx
        .platform_id       = 0x49,    // used as 4th octect in device BLE address
        .device_id         = 0x0007, // TODO: replace with your lab bench number
        .adv_name          = "EE149 LED", // used in advertisements if there is room
        .adv_interval      = MSEC_TO_UNITS(1000, UNIT_0_625_MS),
        .min_conn_interval = MSEC_TO_UNITS(500, UNIT_1_25_MS),
        .max_conn_interval = MSEC_TO_UNITS(1000, UNIT_1_25_MS),
};

// 32e61089-2b22-4db5-a914-43ce41986c70
static simple_ble_service_t led_service = {{
    .uuid128 = {0x70,0x6C,0x98,0x41,0xCE,0x43,0x14,0xA9,
                0xB5,0x4D,0x22,0x2B,0x89,0x10,0xE6,0x32}
}};

static simple_ble_char_t led_state_char = {.uuid16 = 0x1090};
static simple_ble_char_t lcd_state_char = {.uuid16 = 0x1091};
static uint8_t led_state = 0;
static uint8_t lcd_state[185];
// memset(lcd_state, (uint8_t)0, 185);

/*******************************************************************************
 *   State for this application
 ******************************************************************************/
// Main application state
simple_ble_app_t* simple_ble_app;
states state = ACTIVE;
braking_states brake_state = NOT_BRAKE;
bool tilt_begin = false;

void ble_evt_write(ble_evt_t const* p_ble_evt) {
    if (simple_ble_is_char_event(p_ble_evt, &led_state_char)) {
      printf("Got write to LED characteristic!\n");
      printf("%d\n", led_state);
      if (led_state == 1) {
        state = LEFT;
      } else if (led_state == 2) {
        state = RIGHT;
      } else {
        printf("Turning off LED!\n");
        nrf_gpio_pin_set(BUCKLER_LED0);
        nrf_gpio_pin_set(BUCKLER_LED1);
      }
      tilt_begin = false;
    }

    if (simple_ble_is_char_event(p_ble_evt, &lcd_state_char)) {

      display_write(lcd_state, DISPLAY_LINE_1);
    }
}


int main(void) {
  ret_code_t error_code = NRF_SUCCESS;


  // initialize i2c master (two wire interface)
  nrf_drv_twi_config_t i2c_config = NRF_DRV_TWI_DEFAULT_CONFIG;
  i2c_config.scl = BUCKLER_SENSORS_SCL;
  i2c_config.sda = BUCKLER_SENSORS_SDA;
  i2c_config.frequency = NRF_TWIM_FREQ_100K;
  error_code = nrf_twi_mngr_init(&twi_mngr_instance, &i2c_config);
  APP_ERROR_CHECK(error_code);
  mpu9250_init(&twi_mngr_instance);
  printf("MPU9250 initialized\n");
  // Initialize

  // initialize display
  nrf_drv_spi_t spi_instance = NRF_DRV_SPI_INSTANCE(1);
  nrf_drv_spi_config_t spi_config = {
    .sck_pin = BUCKLER_LCD_SCLK,
    .mosi_pin = BUCKLER_LCD_MOSI,
    .miso_pin = BUCKLER_LCD_MISO,
    .ss_pin = BUCKLER_LCD_CS,
    .irq_priority = NRFX_SPI_DEFAULT_CONFIG_IRQ_PRIORITY,
    .orc = 0,
    .frequency = NRF_DRV_SPI_FREQ_4M,
    .mode = NRF_DRV_SPI_MODE_2,
    .bit_order = NRF_DRV_SPI_BIT_ORDER_MSB_FIRST
  };

  error_code = nrf_drv_spi_init(&spi_instance, &spi_config, NULL, NULL);
  APP_ERROR_CHECK(error_code);
  display_init(&spi_instance);
  display_write("Hello, Human!", DISPLAY_LINE_0);
  printf("Display initialized!\n");

  // Setup LED GPIO
  nrf_gpio_cfg_output(BUCKLER_LED0);
  nrf_gpio_cfg_output(BUCKLER_LED1);
  nrf_gpio_cfg_output(BUCKLER_LED2);

  // Setup BLE
  simple_ble_app = simple_ble_init(&ble_config);

  simple_ble_add_service(&led_service);

  simple_ble_add_characteristic(1, 1, 0, 0,
      sizeof(led_state), (uint8_t*)&led_state,
      &led_service, &led_state_char);

  simple_ble_add_characteristic(1, 1, 0, 1,
      sizeof(lcd_state), lcd_state,
      &led_service, &lcd_state_char);

  // Start Advertising
  simple_ble_adv_only_name();
  float y = 0;
  float x = 0;
  while(1) {
    y = mpu9250_read_accelerometer().y_axis;
    x = mpu9250_read_accelerometer().x_axis;
    if (tilt_begin && y < 0.2 && y > -.2) {
      state = ACTIVE;
    } else if (y > 0.2 || y < -0.2) {
      tilt_begin = true;
    }

    if (x < -0.1) {
      brake_state = BRAKE;
    } else {
      brake_state = NOT_BRAKE;
    }

    if (brake_state == BRAKE) {
      printf ("Braking\n");
      nrf_gpio_pin_clear(BUCKLER_LED2);
    } else {
      nrf_gpio_pin_set(BUCKLER_LED2);
    }

    if (state == LEFT) {

      printf("Making a left turn!\n");
      nrf_gpio_pin_toggle(BUCKLER_LED0);
      nrf_delay_ms(500);
    } else if (state == RIGHT) {
      printf("Making a right turn!\n");
        nrf_gpio_pin_toggle(BUCKLER_LED1);
        nrf_delay_ms(500);
    }else {
      nrf_gpio_pin_set(BUCKLER_LED0);
      nrf_gpio_pin_set(BUCKLER_LED1);
      nrf_delay_ms(500);
    }
    
    printf("Reading (accel): %f\n", y);
  }
}

