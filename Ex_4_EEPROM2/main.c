#include "pico/stdlib.h"
#include "hardware/i2c.h"
#include "hardware/pwm.h"
#include <stdio.h>
#include <stdint.h>
#include <string.h>
#include "hardware/uart.h"
#include "hardware/gpio.h"

// EEPROM and I2C configuration
#define I2C_PORT i2c0
#define EEPROM_ADDRESS 0x50
#define SDA_PIN 16
#define SCL_PIN 17
#define EEPROM_HIGHEST_ADDRESS 0x7FFE // Maximum address for storing LED state

#define BUTTON_SW0 9
#define BUTTON_SW1 8
#define BUTTON_SW2 7

#define LED_D1 22
#define LED_D2 21
#define LED_D3 20

#define UART_ID uart0
#define BAUD_RATE 115200
#define UART_TX_PIN 0
#define UART_RX_PIN 1

// Struct for LED state and its inverted state for validation
typedef struct {
    uint8_t state;
    uint8_t not_state;
} ledstate_t;

typedef struct {
    char message[62]; // 61 characters for the message + 1 for null terminator
    uint16_t crc;     // 16-bit CRC for data integrity
} log_entry;

// Function Prototypes
void init_i2c();
void init_buttons();
void init_leds();
void set_leds(uint8_t state);
bool led_state_is_valid(const ledstate_t *ls);
bool write_state_to_eeprom(const ledstate_t *state);
bool read_state_from_eeprom(ledstate_t *state);
bool clear_eeprom();
void init_uart();

int main() {
    stdio_init_all();
    init_i2c();
    init_buttons();
    init_leds();
    init_uart();

    const uint MAX_COMMAND_LENGTH = 64;
    char command_buffer[MAX_COMMAND_LENGTH];
    uint command_length = 0; // Current length of the command buffer

    ledstate_t led_state;
    // Try reading the state from EEPROM and validate it
    if (!read_state_from_eeprom(&led_state) || !led_state_is_valid(&led_state)) {
        // Set default state (middle LED on) if read fails or validation fails
        led_state.state = 0b010;
        led_state.not_state = ~led_state.state;
    }
    set_leds(led_state.state);

    absolute_time_t start_time = get_absolute_time();
    bool last_button_states[3] = {true, true, true};

    while (1) {
        // UART command processing
        if (uart_is_readable(UART_ID)) {
            char ch = uart_getc(UART_ID);
            if (ch == '\n' || ch == '\r') {
                command_buffer[command_length] = '\0'; // Null-terminate the command string
                // Process the command here
                if (strcmp(command_buffer, "read") == 0) {
                    // Read the LED state from EEPROM and print it
                    ledstate_t read_led_state;
                    if (read_state_from_eeprom(&read_led_state)) {
                        printf("LED State: %u, Not State: %u\n", read_led_state.state, read_led_state.not_state);
                    } else {
                        printf("Failed to read from EEPROM.\n");
                    }
                } else if (strcmp(command_buffer, "erase") == 0) {
                    // Reset the EEPROM by writing zeros to the led_state section
                    ledstate_t reset_led_state = {0, 0};
                    if (write_state_to_eeprom(&reset_led_state)) {
                        printf("EEPROM erased.\n");
                    } else {
                        printf("Failed to erase EEPROM.\n");
                    }
                }
                command_length = 0; // Clear the command buffer after processing
            } else if (command_length < MAX_COMMAND_LENGTH - 1) {
                command_buffer[command_length++] = ch; // Add character to buffer and increment length
            } else {
                command_length = 0; // Reset the command buffer on overflow
            }
        }

        // Button state processing
        bool current_button_states[3] = {gpio_get(BUTTON_SW0), gpio_get(BUTTON_SW1), gpio_get(BUTTON_SW2)};
        bool state_changed = false;

        for (int i = 0; i < 3; i++) {
            // Button press is detected
            if (!current_button_states[i] && last_button_states[i]) {
                // Toggle corresponding LED state
                led_state.state ^= (1 << (2 - i));
                state_changed = true;
            }
            // Update last button state
            last_button_states[i] = current_button_states[i];
        }

        // If an LED state change is detected
        if (state_changed) {
            // Update LEDs
            set_leds(led_state.state);
            // Invert state for validation
            led_state.not_state = ~led_state.state;
            // Write new state to EEPROM
            write_state_to_eeprom(&led_state);

            // Optional: Log state change time and status
            int64_t elapsed_time = absolute_time_diff_us(start_time, get_absolute_time()) / 1000000;
            printf("Elapsed Time: %lld s, LEDs: SW_0 %s, SW_1 %s, SW_2 %s\n",
                   elapsed_time, (led_state.state & 0b100) ? "On" : "Off",
                   (led_state.state & 0b010) ? "On" : "Off",
                   (led_state.state & 0b001) ? "On" : "Off");
        }

        // Sleep briefly to debounce
        sleep_ms(10);
    }
    return 0;
}



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

void setup_pwm_for_led(uint gpio) {
    uint slice_num = pwm_gpio_to_slice_num(gpio);
    pwm_config config = pwm_get_default_config();
    pwm_config_set_clkdiv(&config, 125.0); // Set PWM clock divider for 1 MHz base frequency
    pwm_config_set_wrap(&config, 999);     // Set wrap value for 1 kHz PWM frequency
    pwm_init(slice_num, &config, true);    // Initialize PWM with this configuration
    pwm_set_chan_level(slice_num, pwm_gpio_to_channel(gpio), 0);  // Initially off
}

void init_leds() {
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

bool led_state_is_valid(const ledstate_t *ls) {
    return ls->state == (uint8_t)~ls->not_state;
}

bool write_byte(uint16_t memory_address, uint8_t data) {
    uint8_t buffer[3];
    buffer[0] = (memory_address >> 8) & 0xFF; // Memory address high byte
    buffer[1] = memory_address & 0xFF;        // Memory address low byte
    buffer[2] = data;                         // Data byte

    int result = i2c_write_blocking(I2C_PORT, EEPROM_ADDRESS, buffer, 3, false);
    if (result != 3) {
        return false;
    }
    sleep_ms(5);  // EEPROM write cycle time
    return true;
}

// Read a single byte from EEPROM
bool read_byte(uint16_t memory_address, uint8_t *data) {
    uint8_t address_buffer[2];
    address_buffer[0] = (memory_address >> 8) & 0xFF; // Memory address high byte
    address_buffer[1] = memory_address & 0xFF;        // Memory address low byte

    if (i2c_write_blocking(I2C_PORT, EEPROM_ADDRESS, address_buffer, 2, true) != 2) {
        return false;
    }
    if (i2c_read_blocking(I2C_PORT, EEPROM_ADDRESS, data, 1, false) != 1) {
        return false;
    }
    return true;
}

// Write multiple bytes to EEPROM
bool write_bytes(uint16_t start_memory_address, const uint8_t *data, size_t length) {
    for (size_t i = 0; i < length; i++) {
        uint16_t memory_address = start_memory_address + i;
        if (!write_byte(memory_address, data[i])) {
            return false;
        }
    }
    return true;
}

// Read multiple bytes from EEPROM
bool read_bytes(uint16_t start_memory_address, uint8_t *data, size_t length) {
    for (size_t i = 0; i < length; i++) {
        uint16_t memory_address = start_memory_address + i;
        if (!read_byte(memory_address, &data[i])) {
            return false;
        }
    }
    return true;
}

bool write_state_to_eeprom(const ledstate_t *state) {
    if (!state) return false;
    return write_bytes(EEPROM_HIGHEST_ADDRESS, (const uint8_t *)state, sizeof(ledstate_t));
}

bool read_state_from_eeprom(ledstate_t *state) {
    if (!state) return false;
    return read_bytes(EEPROM_HIGHEST_ADDRESS, (uint8_t *)state, sizeof(ledstate_t));
}

bool clear_eeprom() {
    uint8_t invalid_data[sizeof(ledstate_t)] = {0}; // Array of zeros
    return write_bytes(EEPROM_HIGHEST_ADDRESS, invalid_data, sizeof(invalid_data));
}

void init_uart() {
    uart_init(UART_ID, BAUD_RATE);
    gpio_set_function(UART_TX_PIN, GPIO_FUNC_UART);
    gpio_set_function(UART_RX_PIN, GPIO_FUNC_UART);
    uart_set_hw_flow(UART_ID, false, false);
    uart_set_format(UART_ID, 8, 1, UART_PARITY_NONE);
    uart_set_fifo_enabled(UART_ID, false); // Disable FIFO to handle data byte-by-byte
}
