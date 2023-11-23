#include "pico/stdlib.h"
#include "hardware/i2c.h"
#include <stdio.h>

#define I2C_PORT i2c0
#define EEPROM_ADDRESS 0x50
#define SDA_PIN 16
#define SCL_PIN 17

// Initialize I2C
void init_i2c() {
    i2c_init(I2C_PORT, 100 * 1000);  // Initialize I2C at 100 kHz
    gpio_set_function(SDA_PIN, GPIO_FUNC_I2C);
    gpio_set_function(SCL_PIN, GPIO_FUNC_I2C);
    gpio_pull_up(SDA_PIN);
    gpio_pull_up(SCL_PIN);
    printf("I2C Initialized\n");
}

// Write a single byte to EEPROM
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

// Main function
int main() {
    stdio_init_all();
    sleep_ms(10000); // Wait for serial console
    init_i2c();

    printf("I2C EEPROM Interface Example\n");

    // Test writing and reading single bytes
    uint8_t write_data = 0xA5;
    uint8_t read_data = 0;

    if (write_byte(0, write_data)) {
        printf("Written 0x%x to address 0\n", write_data);
    } else {
        printf("Write operation failed\n");
    }

    if (read_byte(0, &read_data)) {
        printf("Read 0x%x from address 0\n", read_data);
    } else {
        printf("Read operation failed\n");
    }

    // Test writing and reading multiple bytes
    uint8_t multi_write_data[] = {0xA5, 0xBC};
    uint8_t multi_read_data[2] = {0};

    if (write_bytes(0, multi_write_data, sizeof(multi_write_data))) {
        printf("Written multiple bytes to address 0 and 1\n");
    } else {
        printf("Multi-byte write operation failed\n");
    }

    if (read_bytes(0, multi_read_data, sizeof(multi_read_data))) {
        printf("Read multiple bytes from address 0 and 1: 0x%x 0x%x\n", multi_read_data[0], multi_read_data[1]);
    } else {
        printf("Multi-byte read operation failed\n");
    }

    return 0;
}
