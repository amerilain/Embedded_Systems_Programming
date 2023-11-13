#include <stdio.h>
#include <string.h>
#include <ctype.h>
#include "pico/stdlib.h"
#include "uart.h"
#include "ring_buffer.h"
#include <hardware/uart.h>
#include <hardware/irq.h>

#define SW_0 9 // Assuming SW_0 is connected to GPIO 9
#define UART_NR uart0
#define BAUD_RATE 9600
#define UART_TX_PIN 0
#define UART_RX_PIN 1

// UART ring buffer
ring_buffer uart_rx_buffer;
uint8_t uart_rx_buffer_data[256]; // Buffer size can be adjusted as needed

void uart_irq_handler() {
    while (uart_is_readable(UART_NR)) {
        uint8_t ch = uart_getc(UART_NR);
        rb_put(&uart_rx_buffer, ch); // Store received data in the ring buffer
    }
}

void uart_init_with_ring_buffer(uart_inst_t *uart, int tx_pin, int rx_pin, int speed) {
    uart_init(uart, speed);
    gpio_set_function(tx_pin, GPIO_FUNC_UART);
    gpio_set_function(rx_pin, GPIO_FUNC_UART);

    // Initialize ring buffer for UART RX
    rb_init(&uart_rx_buffer, uart_rx_buffer_data, sizeof(uart_rx_buffer_data));

    // Enable UART receive interrupt
    uart_set_irq_enables(uart, true, false);

    // Set up the IRQ handler for UART receive interrupts
    irq_set_exclusive_handler(UART0_IRQ, uart_irq_handler);
    irq_set_enabled(UART0_IRQ, true);
}

// Function to read from UART ring buffer with a timeout
int uart_read_timeout_from_ring_buffer(uart_inst_t *uart, uint8_t *buffer, int size, uint32_t timeout_ms) {
    int count = 0;
    uint32_t start_time = to_ms_since_boot(get_absolute_time());

    while (count < size && (to_ms_since_boot(get_absolute_time()) - start_time) < timeout_ms) {
        if (rb_empty(&uart_rx_buffer)) {
            sleep_ms(10);
            continue;
        }
        buffer[count++] = rb_get(&uart_rx_buffer);
    }

    return count; // Number of bytes read
}

void read_firmware_version() {
    char ver_command[] = "AT+VER\r\n";
    char response[30]; // Buffer to store the response

    uart_puts(UART_NR, ver_command);
    int len = uart_read_timeout_from_ring_buffer(UART_NR, (uint8_t *)response, sizeof(response), 500);
    response[len] = '\0';

    if (len > 0) {
        printf("Firmware Version: %s\n", response);
    } else {
        printf("Module stopped responding\n");
    }
}

void read_and_process_deveui() {
    char id_command[] = "AT+ID=DevEui\r\n";
    char response[50]; // Buffer to store the response

    uart_puts(UART_NR, id_command);
    int len = uart_read_timeout_from_ring_buffer(UART_NR, (uint8_t *)response, sizeof(response), 500);
    response[len] = '\0';

    if (len > 0) {
        // Find the start of the DevEui in the response
        char *devEuiStart = strstr(response, "DevEui,");
        if (devEuiStart != NULL) {
            devEuiStart += 8; // Skip past "DevEui,"

            // Process the DevEui (remove colons and convert to lowercase)
            char processed_devEui[50];
            int j = 0;
            for (int i = 0; devEuiStart[i] != '\0' && devEuiStart[i] != '\r' && devEuiStart[i] != '\n'; ++i) {
                if (devEuiStart[i] != ':') {
                    processed_devEui[j++] = tolower(devEuiStart[i]);
                }
            }
            processed_devEui[j] = '\0';

            printf("Processed DevEui: %s\n", processed_devEui);
        } else {
            printf("DevEui not found in response\n");
        }
    } else {
        printf("Module stopped responding\n");
    }
}


int main() {
    stdio_init_all();
    gpio_init(SW_0);
    gpio_set_dir(SW_0, GPIO_IN);
    gpio_pull_up(SW_0);

    // Initialize UART with ring buffer
    uart_init_with_ring_buffer(UART_NR, UART_TX_PIN, UART_RX_PIN, BAUD_RATE);

    while (1) {
        // Wait for SW_0 press
        while (gpio_get(SW_0)) {
            sleep_ms(10); // Debounce delay
        }

        // Establish communication
        char at_command[] = "AT\r\n";
        char response[20];
        bool connected = false;

        for (int i = 0; i < 5 && !connected; i++) {
            uart_puts(UART_NR, at_command);
            sleep_ms(500);

            int len = uart_read_timeout_from_ring_buffer(UART_NR, (uint8_t *)response, sizeof(response), 500);
            response[len] = '\0'; // Null-terminate the string

            if (strstr(response, "+AT: OK") != NULL) {
                printf("Connected to LoRa module\n");
                connected = true;
            }
        }

        if (!connected) {
            printf("Module not responding\n");
            continue; // Go back to the beginning of the loop
        }

        // Read firmware version
        read_firmware_version();

        // Read and process DevEui
        read_and_process_deveui();
    }

    return 0;
}
