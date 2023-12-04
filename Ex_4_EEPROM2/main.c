#include "pico/stdlib.h"
#include "hardware/i2c.h"
#include "hardware/pwm.h"
#include <stdio.h>
#include <string.h>
#include "hardware/uart.h"
#include "hardware/gpio.h"

// EEPROM and I2C configuration
#define I2C_PORT i2c0
#define EEPROM_ADDRESS 0x50
#define SDA_PIN 16
#define SCL_PIN 17
#define EEPROM_HIGHEST_ADDRESS 0x7FFE
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
#define LOG_START_ADDRESS 0x0000
#define MAX_LOG_ENTRIES (2048 / sizeof(log_entry))

typedef struct {
    uint8_t state;
    uint8_t not_state;
} ledstate_t;

typedef struct {
    char message[62]; // 61 characters + 1 for null terminator
    uint16_t crc;
} log_entry;

int current_log_index = 0;

void init_i2c();
void init_buttons();
void init_leds();
void set_leds(uint8_t state);
bool led_state_is_valid(const ledstate_t *ls);
bool read_state_from_eeprom(ledstate_t *state);
void init_uart();
uint16_t crc16(const uint8_t *data_p, unsigned length);
void write_to_log(const char *message);
void read_log();
void erase_log();
void init_log();
bool write_log_entry_to_eeprom(const char* command);
void led_state_to_string(const ledstate_t *led_state, char *state_str);


int main() {
    stdio_init_all();
    init_i2c();
    init_buttons();
    init_leds();
    init_uart();
    init_log();


    const uint MAX_COMMAND_LENGTH = 64;
    char command_buffer[MAX_COMMAND_LENGTH];
    uint command_length = 0; // Current length of the command buffer
    const uint DEBOUNCE_DURATION_MS = 20; // Duration for debouncing in milliseconds

    ledstate_t led_state;
    // Try reading the state from EEPROM and validate it
    if (!read_state_from_eeprom(&led_state) || !led_state_is_valid(&led_state)) {
        // Set default state (middle LED on) if read fails or validation fails
        led_state.state = 0b010; // Only middle LED on
        led_state.not_state = ~led_state.state;
    }
    set_leds(led_state.state);

    absolute_time_t start_time = get_absolute_time();
    bool last_button_states[3] = {true, true, true};


    while (true) {
        // UART reading and command processing
        while (uart_is_readable(UART_ID)) {
            char ch = uart_getc(UART_ID);
            //uart_putc(UART_ID, ch); // Echo back the character

            if (ch == '\n' || ch == '\r') {
                if (command_length > 0) {
                    command_buffer[command_length] = '\0'; // Null-terminate the command string
                    if (strcmp(command_buffer, "read") == 0) {
                        read_log();
                    } else if (strcmp(command_buffer, "erase") == 0) {
                        erase_log();
                        printf("Log erased.\n");
                    }
                    command_length = 0;
                }
            } else if (command_length < MAX_COMMAND_LENGTH - 1) {
                command_buffer[command_length++] = ch; // Add character to buffer
            }
        }

        // Handle button states and LED control
        bool current_button_states[3] = {gpio_get(BUTTON_SW0), gpio_get(BUTTON_SW1), gpio_get(BUTTON_SW2)};
        bool state_changed = false;

        int button_pins[3] = {BUTTON_SW0, BUTTON_SW1, BUTTON_SW2};

        for (int i = 0; i < 3; i++) {
            if (!current_button_states[i] && last_button_states[i]) {
                // Wait for a short period to debounce
                sleep_ms(DEBOUNCE_DURATION_MS);
                bool button_state_after_delay = gpio_get(button_pins[i]);

                if (!button_state_after_delay) {
                    // Button press is considered valid
                    led_state.state ^= (1 << (2 - i));
                    state_changed = true;
                }
            }
            last_button_states[i] = current_button_states[i];
        }

        if (state_changed) {
            set_leds(led_state.state);
            led_state.not_state = ~led_state.state;

            char led_state_str[62];
            led_state_to_string(&led_state, led_state_str);
            write_log_entry_to_eeprom(led_state_str);

            int64_t elapsed_time = absolute_time_diff_us(start_time, get_absolute_time()) / 1000000;
            printf("Elapsed Time: %lld s, LEDs: SW_0 %s, SW_1 %s, SW_2 %s\n",
                   elapsed_time, (led_state.state & 0b100) ? "On" : "Off",
                   (led_state.state & 0b010) ? "On" : "Off",
                   (led_state.state & 0b001) ? "On" : "Off");
        }

        sleep_ms(.5);
    }

    return 0;
}



void init_i2c() {
    i2c_init(I2C_PORT, 100 * 1000);  // Initialize I2C at 100 kHz
    gpio_set_function(SDA_PIN, GPIO_FUNC_I2C);
    gpio_set_function(SCL_PIN, GPIO_FUNC_I2C);
    gpio_pull_up(SDA_PIN);
    gpio_pull_up(SCL_PIN);
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
    pwm_config_set_clkdiv(&config, 125.0);
    pwm_config_set_wrap(&config, 999);
    pwm_init(slice_num, &config, true);
    pwm_set_chan_level(slice_num, pwm_gpio_to_channel(gpio), 0);
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
    uint level_d1 = (state & 0b001) ? 500 : 0;
    uint level_d2 = (state & 0b010) ? 500 : 0;
    uint level_d3 = (state & 0b100) ? 500 : 0;

    pwm_set_chan_level(pwm_gpio_to_slice_num(LED_D1), pwm_gpio_to_channel(LED_D1), level_d1);
    pwm_set_chan_level(pwm_gpio_to_slice_num(LED_D2), pwm_gpio_to_channel(LED_D2), level_d2);
    pwm_set_chan_level(pwm_gpio_to_slice_num(LED_D3), pwm_gpio_to_channel(LED_D3), level_d3);
}

bool led_state_is_valid(const ledstate_t *ls) {
    return ls->state == (uint8_t)~ls->not_state;
}

bool write_byte(uint16_t memory_address, uint8_t data) {
    uint8_t buffer[3];
    buffer[0] = (memory_address >> 8) & 0xFF; // high byte
    buffer[1] = memory_address & 0xFF;        // low byte
    buffer[2] = data;                         // Data byte

    int result = i2c_write_blocking(I2C_PORT, EEPROM_ADDRESS, buffer, 3, false);
    if (result != 3) {
        return false;
    }
    sleep_ms(5);
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

bool write_bytes(uint16_t start_memory_address, const uint8_t *data, size_t length) {
    for (size_t i = 0; i < length; i++) {
        uint16_t memory_address = start_memory_address + i;
        if (!write_byte(memory_address, data[i])) {
            printf("Failed to write data at address %u\n", memory_address);
            return false;
        }
    }
    // printf("Data written successfully at address %u.\n", start_memory_address);
    return true;
}


bool read_bytes(uint16_t start_memory_address, uint8_t *data, size_t length) {
    for (size_t i = 0; i < length; i++) {
        uint16_t memory_address = start_memory_address + i;
        if (!read_byte(memory_address, &data[i])) {
            printf("Failed to read data at address %u\n", memory_address);
            return false;
        }
    }
    // printf("Data read successfully from address %u.\n", start_memory_address);
    return true;
}


bool write_log_entry_to_eeprom(const char* message) {
    log_entry entry;
    strncpy(entry.message, message, 61);  // Ensure it's null-terminated
    entry.message[61] = '\0';
    entry.crc = crc16((const uint8_t*)entry.message, strlen(entry.message));

    uint16_t write_address = LOG_START_ADDRESS + (current_log_index * sizeof(log_entry));
    if (write_address >= (LOG_START_ADDRESS + MAX_LOG_ENTRIES * sizeof(log_entry))) {
        // Log is full, handle as needed (e.g., overwrite or stop logging)
        return false; // For now, stop logging when full
    }

    uint8_t *byte_ptr = (uint8_t *)&entry;
    bool write_success = write_bytes(write_address, byte_ptr, sizeof(log_entry));
    if (write_success) {
        current_log_index = (current_log_index + 1) % MAX_LOG_ENTRIES; // Increment and wrap around
    }
    return write_success;
}

bool read_state_from_eeprom(ledstate_t *state) {
    if (!state) return false;
    return read_bytes(EEPROM_HIGHEST_ADDRESS, (uint8_t *)state, sizeof(ledstate_t));
}

void init_uart() {
    uart_init(UART_ID, BAUD_RATE);
    gpio_set_function(UART_TX_PIN, GPIO_FUNC_UART);
    gpio_set_function(UART_RX_PIN, GPIO_FUNC_UART);
    uart_set_hw_flow(UART_ID, false, false);
    uart_set_format(UART_ID, 8, 1, UART_PARITY_NONE);
    uart_set_fifo_enabled(UART_ID, false); // Disable FIFO to handle data byte-by-byte
}

uint16_t crc16(const uint8_t *data_p, size_t length) {
    uint8_t x;
    uint16_t crc = 0xFFFF;
    while (length--) {
        //printf("%02x ", *data_p);  // Print each byte of data
        x = crc >> 8 ^ *data_p++;
        x ^= x >> 4;
        crc = (crc << 8) ^ ((uint16_t)(x << 12)) ^ ((uint16_t)(x << 5)) ^ ((uint16_t)x);
    }
    //printf("\nCalculated CRC: %u\n", crc);  // Print the final CRC value
    return crc;
}

void write_to_log(const char *message) {
    log_entry entry;
    strncpy(entry.message, message, 61); // Copy the message
    entry.message[61] = '\0'; // Ensure null termination

    entry.crc = crc16((const uint8_t *)entry.message, strlen(entry.message));

    uint16_t write_address = LOG_START_ADDRESS + (current_log_index * sizeof(log_entry));
    if (write_address >= (LOG_START_ADDRESS + MAX_LOG_ENTRIES * sizeof(log_entry))) {
        current_log_index = 0; // Reset index if at the end
        write_address = LOG_START_ADDRESS;
    }

    uint8_t *byte_ptr = (uint8_t *)&entry;
    bool success = write_bytes(write_address, byte_ptr, sizeof(log_entry));
    if (success) {
        printf("Log written: '%s' at address %u\n", entry.message, write_address);
    } else {
        printf("Failed to write log at address %u\n", write_address);
    }
    current_log_index++; // Increment the log index
}


void read_log() {
    for (int i = 0; i < MAX_LOG_ENTRIES; i++) {
        uint16_t read_address = LOG_START_ADDRESS + (i * sizeof(log_entry));
        log_entry entry;
        read_bytes(read_address, (uint8_t *)&entry, sizeof(log_entry));

        // Check if this is the end of the log
        if (entry.message[0] == 0) {
            printf("End of log detected at address %u\n", read_address);
            break;
        }

        uint16_t calculated_crc = crc16((const uint8_t *)entry.message, strlen(entry.message));

        // Only print the log entry if it is valid
        if (calculated_crc == entry.crc) {
            printf("Read Log at address %u: %s\n", read_address, entry.message);
        } else {
            printf("Invalid log entry detected at address %u. Stopping read.\n", read_address);
            break;
        }
    }
}

void erase_log() {
    log_entry empty_entry;
    memset(&empty_entry, 0, sizeof(log_entry)); // Set all bytes of empty_entry to 0

    for (int i = 0; i < MAX_LOG_ENTRIES; i++) {
        uint16_t write_address = LOG_START_ADDRESS + (i * sizeof(log_entry));
        write_bytes(write_address, (const uint8_t *)&empty_entry, sizeof(log_entry));
    }

    current_log_index = 0; // Reset the current log index
}


void init_log() {
    write_to_log("Boot");
}

void led_state_to_string(const ledstate_t *led_state, char *state_str) {
    snprintf(state_str, 62, "LEDs: SW_0 %s, SW_1 %s, SW_2 %s",
             (led_state->state & 0b100) ? "On" : "Off",
             (led_state->state & 0b010) ? "On" : "Off",
             (led_state->state & 0b001) ? "On" : "Off");
}





