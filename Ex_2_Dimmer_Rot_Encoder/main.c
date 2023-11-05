#include "pico/stdlib.h"
#include "hardware/pwm.h"
#include "hardware/irq.h"
#include "hardware/gpio.h"

#define ROT_A 10
#define ROT_B 11
#define ROT_SW 12
#define LED_D1 22
#define LED_D2 21
#define LED_D3 20
#define MIN_BRIGHTNESS 0
#define MAX_BRIGHTNESS 1000
#define HALF_BRIGHTNESS 500

volatile int brightness = MIN_BRIGHTNESS;
volatile bool leds_on = false;
volatile bool first_start = true;
bool prev_button_sw_state = true;
uint64_t last_button_press_time = 0;

void setup_pwm(uint gpio);
void set_leds(bool state);
void rotary_encoder_isr();
void update_pwm_brightness();

int main() {
    stdio_init_all();

    // Set up the buttons and rotary encoder
    gpio_init(ROT_SW);
    gpio_set_dir(ROT_SW, GPIO_IN);
    gpio_pull_up(ROT_SW); // Enable internal pull-up for the switch

    gpio_init(ROT_A);
    gpio_set_dir(ROT_A, GPIO_IN); // No pull-up/pull-down

    gpio_init(ROT_B);
    gpio_set_dir(ROT_B, GPIO_IN); // No pull-up/pull-down

    // Set up the LEDs
    gpio_set_function(LED_D1, GPIO_FUNC_PWM);
    gpio_set_function(LED_D2, GPIO_FUNC_PWM);
    gpio_set_function(LED_D3, GPIO_FUNC_PWM);
    setup_pwm(LED_D1);
    setup_pwm(LED_D2);
    setup_pwm(LED_D3);

    // Initialize system with lights off and brightness at minimum
    set_leds(false);
    update_pwm_brightness();

    // Enable interrupts for the rotary encoder
    gpio_set_irq_enabled_with_callback(ROT_A, GPIO_IRQ_EDGE_RISE, true, &rotary_encoder_isr);

    while (true) {
        // Toggle LEDs with rotary encoder switch
        bool current_button_sw_state = gpio_get(ROT_SW);
        uint64_t current_time = to_us_since_boot(get_absolute_time());

        // Check for button press with debounce
        if (!current_button_sw_state && prev_button_sw_state && (current_time - last_button_press_time > 50000)) {
            last_button_press_time = current_time;
            // Button is currently pressed, wait for it to be released
            while (!gpio_get(ROT_SW)) {
                tight_loop_contents();
            }
            // If brightness is at 0%, set brightness to 50%
            if (brightness == MIN_BRIGHTNESS) {
                brightness = HALF_BRIGHTNESS;
                leds_on = true; // Ensure LEDs are on
            } else {
                // Toggle LEDs on release
                leds_on = !leds_on;
            }
            first_start = false;
            set_leds(leds_on);
        }

        prev_button_sw_state = current_button_sw_state;  // Update the previous button state

        sleep_ms(10);
    }

    return 0;
}

void setup_pwm(uint gpio) {
    uint slice_num = pwm_gpio_to_slice_num(gpio);
    pwm_config config = pwm_get_default_config();
    pwm_config_set_clkdiv(&config, 125.0); // Set the PWM clock divider to achieve 1 MHz base frequency.
    pwm_config_set_wrap(&config, 999);  // Set wrap value to achieve 1 kHz PWM frequency
    pwm_init(slice_num, &config, true);  // Start PWM after initialization
    pwm_set_chan_level(slice_num, pwm_gpio_to_channel(gpio), brightness);  // Set the initial duty cycle
}

void set_leds(bool state) {
    leds_on = state;
    update_pwm_brightness();
}

void rotary_encoder_isr() {
    // Read the current state of ROT_B
    bool rot_b_state = gpio_get(ROT_B);

    // Ignore rotary encoder if it's the first start and LEDs are off
    if (first_start && !leds_on) {
        return;
    }

    // If the LEDs are off and the brightness is at minimum, allow the knob to increase brightness
    if (!leds_on && brightness == MIN_BRIGHTNESS) {
        if (!rot_b_state) {
            // Clockwise turn
            brightness += 5;
            // Automatically turn on the LEDs if the knob is turned
            leds_on = true;
        }
    }
        // If the LEDs are on, adjust the brightness based on rotation
    else if (leds_on) {
        if (!rot_b_state) {
            // Clockwise turn
            brightness += 5;
            if (brightness > MAX_BRIGHTNESS) {
                brightness = MAX_BRIGHTNESS;
            }
        } else {
            // Counter-clockwise turn
            brightness -= 5;
            if (brightness <= MIN_BRIGHTNESS) {
                brightness = MIN_BRIGHTNESS;
                // Do not turn off the LEDs, just set brightness to minimum
            }
        }
    }

    // Update the PWM brightness if the LEDs are supposed to be on
    if (leds_on) {
        update_pwm_brightness();
    }
}

void update_pwm_brightness() {
    uint slice_num_d1 = pwm_gpio_to_slice_num(LED_D1);
    uint slice_num_d2 = pwm_gpio_to_slice_num(LED_D2);
    uint slice_num_d3 = pwm_gpio_to_slice_num(LED_D3);

    if (leds_on) {
        pwm_set_chan_level(slice_num_d1, pwm_gpio_to_channel(LED_D1), brightness);
        pwm_set_chan_level(slice_num_d2, pwm_gpio_to_channel(LED_D2), brightness);
        pwm_set_chan_level(slice_num_d3, pwm_gpio_to_channel(LED_D3), brightness);
    } else {
        pwm_set_chan_level(slice_num_d1, pwm_gpio_to_channel(LED_D1), MIN_BRIGHTNESS);
        pwm_set_chan_level(slice_num_d2, pwm_gpio_to_channel(LED_D2), MIN_BRIGHTNESS);
        pwm_set_chan_level(slice_num_d3, pwm_gpio_to_channel(LED_D3), MIN_BRIGHTNESS);
    }
}
