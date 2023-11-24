#include "pico/stdlib.h"
#include "hardware/i2c.h"
#include "hardware/pwm.h"
#include <stdio.h>

// EEPROM and I2C configuration
#define I2C_PORT i2c0
#define EEPROM_ADDRESS 0x50
#define SDA_PIN 16
#define SCL_PIN 17
#define EEPROM_HIGHEST_ADDRESS 0x7FFE  // One less than the max to fit the 2-byte structure


#define BUTTON_SW0 9
#define BUTTON_SW1 8
#define BUTTON_SW2 7

#define LED_D1 22
#define LED_D2 21
#define LED_D3 20

// Function Prototypes
void init_i2c();
void init_buttons();
void init_leds();
void set_leds(uint8_t state);
bool write_byte(uint16_t memory_address, uint8_t data);
bool read_byte(uint16_t memory_address, uint8_t *data);
bool write_bytes(uint16_t start_memory_address, const uint8_t *data, size_t length);
bool read_bytes(uint16_t start_memory_address, uint8_t *data, size_t length);

// Main function
int main() {
    stdio_init_all();
    init_i2c(); // Initialize I2C for EEPROM
    init_buttons(); // Initialize buttons
    init_leds(); // Initialize LEDs

    // Define a structure for LED state and validation
    typedef struct {
        uint8_t state;
        uint8_t not_state; // Inverted state for validation
    } ledstate_t;

    // Function to validate LED state
    bool led_state_is_valid(ledstate_t *ls) {
        return ls->state == (uint8_t)~ls->not_state;
    }

    // Load and validate LED state from EEPROM
    ledstate_t led_state;
    if (read_bytes(EEPROM_HIGHEST_ADDRESS, (uint8_t *)&led_state, sizeof(ledstate_t)) &&
        led_state_is_valid(&led_state)) {
        set_leds(led_state.state); // Set LEDs based on stored state
    } else {
        led_state.state = 0b010; // Default state: middle LED on
        led_state.not_state = ~led_state.state;
    }

    absolute_time_t start_time = get_absolute_time();

    bool last_button_states[3] = {true, true, true}; // Store the last states of buttons

    // Main loop
    while (true) {
        bool current_button_states[3] = {
                gpio_get(BUTTON_SW0),
                gpio_get(BUTTON_SW1),
                gpio_get(BUTTON_SW2)
        };

        // Check each button and control corresponding LED
        if (!current_button_states[0] && last_button_states[0]) {
            // Toggle LED D3 for SW_0
            led_state.state ^= 0b100;
        }
        if (!current_button_states[1] && last_button_states[1]) {
            // Toggle LED D2 for SW_1
            led_state.state ^= 0b010;
        }
        if (!current_button_states[2] && last_button_states[2]) {
            // Toggle LED D1 for SW_2
            led_state.state ^= 0b001;
        }

        // Update the LEDs only if a button state changed
        if (current_button_states[0] != last_button_states[0] ||
            current_button_states[1] != last_button_states[1] ||
            current_button_states[2] != last_button_states[2]) {

            set_leds(led_state.state);
            led_state.not_state = ~led_state.state;
            write_bytes(EEPROM_HIGHEST_ADDRESS, (uint8_t *)&led_state, sizeof(ledstate_t));

            int64_t elapsed_time = absolute_time_diff_us(start_time, get_absolute_time()) / 1000000;
            printf("Time: %lld seconds, LED state: 0x%x\n", elapsed_time, led_state.state);

            // Update the last button states
            for (int i = 0; i < 3; i++) {
                last_button_states[i] = current_button_states[i];
            }

            sleep_ms(200); // Debounce delay
        }

        sleep_ms(10); // Main loop delay
    }

    return 0;
}



// Initialize I2C
void init_i2c() {
    i2c_init(I2C_PORT, 100 * 1000);  // Initialize I2C at 100 kHz
    gpio_set_function(SDA_PIN, GPIO_FUNC_I2C);
    gpio_set_function(SCL_PIN, GPIO_FUNC_I2C);
    gpio_pull_up(SDA_PIN);
    gpio_pull_up(SCL_PIN);
    printf("I2C Initialized\n");
}

void init_buttons() {
    gpio_init(BUTTON_SW0);
    gpio_set_dir(BUTTON_SW0, GPIO_IN);
    gpio_pull_up(BUTTON_SW0);
    gpio_init(BUTTON_SW1);
    gpio_set_dir(BUTTON_SW1, GPIO_IN);
    gpio_pull_up(BUTTON_SW1);
    gpio_init(BUTTON_SW2);
    gpio_set_dir(BUTTON_SW2, GPIO_IN);
    gpio_pull_up(BUTTON_SW2);
}

void init_leds() {
    // Function to setup PWM for a single LED
    void setup_pwm_for_led(uint gpio) {
        uint slice_num = pwm_gpio_to_slice_num(gpio);
        pwm_config config = pwm_get_default_config();
        pwm_config_set_clkdiv(&config, 125.0); // Set PWM clock divider for 1 MHz base frequency
        pwm_config_set_wrap(&config, 999);     // Set wrap value for 1 kHz PWM frequency
        pwm_init(slice_num, &config, true);    // Initialize PWM with this configuration
        pwm_set_chan_level(slice_num, pwm_gpio_to_channel(gpio), 0);  // Initially off
    }

    // Set GPIO function for LEDs and initialize PWM
    gpio_set_function(LED_D1, GPIO_FUNC_PWM);
    setup_pwm_for_led(LED_D1);

    gpio_set_function(LED_D2, GPIO_FUNC_PWM);
    setup_pwm_for_led(LED_D2);

    gpio_set_function(LED_D3, GPIO_FUNC_PWM);
    setup_pwm_for_led(LED_D3);
}


void set_leds(uint8_t state) {
    uint level_d1 = (state & 0b001) ? 500 : 0; // Check state of LED D1
    uint level_d2 = (state & 0b010) ? 500 : 0; // Check state of LED D2
    uint level_d3 = (state & 0b100) ? 500 : 0; // Check state of LED D3

    pwm_set_chan_level(pwm_gpio_to_slice_num(LED_D1), pwm_gpio_to_channel(LED_D1), level_d1);
    pwm_set_chan_level(pwm_gpio_to_slice_num(LED_D2), pwm_gpio_to_channel(LED_D2), level_d2);
    pwm_set_chan_level(pwm_gpio_to_slice_num(LED_D3), pwm_gpio_to_channel(LED_D3), level_d3);
}


bool write_byte(uint16_t memory_address, uint8_t data) {
    uint8_t buffer[3];
    buffer[0] = (memory_address >> 8) & 0xFF; // Memory address high byte
    buffer[1] = memory_address & 0xFF;        // Memory address low byte
    buffer[2] = data;                         // Data byte

    printf("Writing 0x%x to EEPROM at address 0x%x\n", data, memory_address);
    int result = i2c_write_blocking(I2C_PORT, EEPROM_ADDRESS, buffer, 3, false);
    if (result != 3) {
        printf("Write failed, result: %d\n", result);
        return false;
    }
    printf("Write successful\n");
    sleep_ms(5);  // EEPROM write cycle time
    return true;
}

// Read a single byte from EEPROM
bool read_byte(uint16_t memory_address, uint8_t *data) {
    uint8_t address_buffer[2];
    address_buffer[0] = (memory_address >> 8) & 0xFF; // Memory address high byte
    address_buffer[1] = memory_address & 0xFF;        // Memory address low byte

    printf("Setting EEPROM read address to 0x%x\n", memory_address);
    if (i2c_write_blocking(I2C_PORT, EEPROM_ADDRESS, address_buffer, 2, true) != 2) {
        printf("Failed to set read address\n");
        return false;
    }

    printf("Reading from EEPROM\n");
    if (i2c_read_blocking(I2C_PORT, EEPROM_ADDRESS, data, 1, false) != 1) {
        printf("Read failed\n");
        return false;
    }
    printf("Read successful, data: 0x%x\n", *data);
    return true;
}

// Write multiple bytes to EEPROM
bool write_bytes(uint16_t start_memory_address, const uint8_t *data, size_t length) {
    for (size_t i = 0; i < length; i++) {
        uint16_t memory_address = start_memory_address + i;
        if (!write_byte(memory_address, data[i])) {
            printf("Multi-byte write failed at address 0x%x\n", memory_address);
            return false;
        }
    }
    printf("Multi-byte write successful\n");
    return true;
}

// Read multiple bytes from EEPROM
bool read_bytes(uint16_t start_memory_address, uint8_t *data, size_t length) {
    for (size_t i = 0; i < length; i++) {
        uint16_t memory_address = start_memory_address + i;
        if (!read_byte(memory_address, &data[i])) {
            printf("Multi-byte read failed at address 0x%x\n", memory_address);
            return false;
        }
    }
    printf("Multi-byte read successful\n");
    return true;
}