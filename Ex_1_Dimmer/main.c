#include "pico/stdlib.h"
#include "hardware/pwm.h"

#define BUTTON_SW0 9
#define BUTTON_SW1 8
#define BUTTON_SW2 7

#define LED_D1 22
#define LED_D2 21
#define LED_D3 20

#define MIN_BRIGHTNESS 0
#define MAX_BRIGHTNESS 1000
#define INITIAL_BRIGHTNESS 500  // 50% brightness

int brightness = INITIAL_BRIGHTNESS;
bool leds_on = false;
bool brightness_zero = false;
bool prev_button_sw1_state = true;

void setup_pwm(uint gpio) {
    uint slice_num = pwm_gpio_to_slice_num(gpio);
    pwm_config config = pwm_get_default_config();
    pwm_config_set_clkdiv(&config, 125.0); // Set the PWM clock divider to achieve 1 MHz base frequency.
    pwm_config_set_wrap(&config, 999);  // Set wrap value to achieve 1 kHz PWM frequency
    pwm_init(slice_num, &config, false);  // false means PWM is not running after initialization
    pwm_set_chan_level(slice_num, pwm_gpio_to_channel(gpio), brightness);  // Set the initial duty cycle
    pwm_set_enabled(slice_num, true);
}

void set_leds(bool state) {
    leds_on = state;
    uint slice_num_d1 = pwm_gpio_to_slice_num(LED_D1);
    uint slice_num_d2 = pwm_gpio_to_slice_num(LED_D2);
    uint slice_num_d3 = pwm_gpio_to_slice_num(LED_D3);

    if (leds_on) {
        if (brightness == MIN_BRIGHTNESS) {
            brightness = INITIAL_BRIGHTNESS;  // If dimmed to 0%, set to 50%
        }
        pwm_set_chan_level(slice_num_d1, pwm_gpio_to_channel(LED_D1), brightness);
        pwm_set_chan_level(slice_num_d2, pwm_gpio_to_channel(LED_D2), brightness);
        pwm_set_chan_level(slice_num_d3, pwm_gpio_to_channel(LED_D3), brightness);
    } else {
        pwm_set_chan_level(slice_num_d1, pwm_gpio_to_channel(LED_D1), MIN_BRIGHTNESS);
        pwm_set_chan_level(slice_num_d2, pwm_gpio_to_channel(LED_D2), MIN_BRIGHTNESS);
        pwm_set_chan_level(slice_num_d3, pwm_gpio_to_channel(LED_D3), MIN_BRIGHTNESS);
    }
}

int main() {
    stdio_init_all();

    // Set up the buttons with pull-ups
    gpio_init(BUTTON_SW0);
    gpio_init(BUTTON_SW1);
    gpio_init(BUTTON_SW2);
    gpio_set_dir(BUTTON_SW0, GPIO_IN);
    gpio_set_dir(BUTTON_SW1, GPIO_IN);
    gpio_set_dir(BUTTON_SW2, GPIO_IN);
    gpio_pull_up(BUTTON_SW0);
    gpio_pull_up(BUTTON_SW1);
    gpio_pull_up(BUTTON_SW2);

    // Set up the LEDs
    gpio_set_function(LED_D1, GPIO_FUNC_PWM);
    gpio_set_function(LED_D2, GPIO_FUNC_PWM);
    gpio_set_function(LED_D3, GPIO_FUNC_PWM);
    setup_pwm(LED_D1);
    setup_pwm(LED_D2);
    setup_pwm(LED_D3);

    // Make sure that LEDs start in the same state
    pwm_set_chan_level(pwm_gpio_to_slice_num(LED_D1), pwm_gpio_to_channel(LED_D1), leds_on ? brightness : MIN_BRIGHTNESS);
    pwm_set_chan_level(pwm_gpio_to_slice_num(LED_D2), pwm_gpio_to_channel(LED_D2), leds_on ? brightness : MIN_BRIGHTNESS);
    pwm_set_chan_level(pwm_gpio_to_slice_num(LED_D3), pwm_gpio_to_channel(LED_D3), leds_on ? brightness : MIN_BRIGHTNESS);

    while (true) {
        // Toggle LEDs
        bool current_button_sw1_state = gpio_get(BUTTON_SW1);

        // Check for button press
        if (!current_button_sw1_state && prev_button_sw1_state) {
            // Button is currently pressed, wait for it to be released
            while (!gpio_get(BUTTON_SW1)) {
                sleep_ms(10);
            }

            set_leds(!leds_on); // Toggle LEDs on release
            brightness_zero = false;  // Reset the brightness_zero flag when LEDs are toggled
        }

        prev_button_sw1_state = current_button_sw1_state;  // Update the previous button state

        // Increase brightness
        if (!gpio_get(BUTTON_SW0) && (leds_on || brightness_zero)) {
            brightness += 5;
            if (brightness > MAX_BRIGHTNESS) {
                brightness = MAX_BRIGHTNESS;
            }
            pwm_set_chan_level(pwm_gpio_to_slice_num(LED_D1), pwm_gpio_to_channel(LED_D1), brightness);
            pwm_set_chan_level(pwm_gpio_to_slice_num(LED_D2), pwm_gpio_to_channel(LED_D2), brightness);
            pwm_set_chan_level(pwm_gpio_to_slice_num(LED_D3), pwm_gpio_to_channel(LED_D3), brightness);
            sleep_ms(20);
            if (brightness_zero && brightness > MIN_BRIGHTNESS) {
                leds_on = true;  // Turn on the LEDs if they were off due to brightness being 0%
                brightness_zero = false;
            }
        }

        // Decrease brightness
        if (!gpio_get(BUTTON_SW2) && leds_on) {  // Only decrease brightness if LEDs are on
            brightness -= 5;
            if (brightness <= MIN_BRIGHTNESS) {
                brightness = MIN_BRIGHTNESS;
                leds_on = false;  // Update the leds_on variable when brightness is 0%
                brightness_zero = true;  // Set the brightness_zero flag
            }
            pwm_set_chan_level(pwm_gpio_to_slice_num(LED_D1), pwm_gpio_to_channel(LED_D1), brightness);
            pwm_set_chan_level(pwm_gpio_to_slice_num(LED_D2), pwm_gpio_to_channel(LED_D2), brightness);
            pwm_set_chan_level(pwm_gpio_to_slice_num(LED_D3), pwm_gpio_to_channel(LED_D3), brightness);
            sleep_ms(20);
        }

        sleep_ms(10);
    }

    return 0;
}