#include "hardware/clocks.h"
#include "hardware/gpio.h"
#include "pico/stdlib.h"
#include "pio_usb.h"

enum {
  USB_HOST_DP_PIN = 16,
  USB_HOST_POWER_PIN = 18,
};

int main(void) {
  // Pico-PIO-USB requires clk_sys to be an integer multiple of 12 MHz.
  set_sys_clock_khz(120000, true);

  gpio_init(USB_HOST_POWER_PIN);
  gpio_set_dir(USB_HOST_POWER_PIN, GPIO_OUT);
  gpio_put(USB_HOST_POWER_PIN, true);
  sleep_ms(100);

  pio_usb_configuration_t config = PIO_USB_DEFAULT_CONFIG;
  config.pin_dp = USB_HOST_DP_PIN;
  pio_usb_host_init(&config);

  while (true) {
    pio_usb_host_task();
    tight_loop_contents();
  }
}
