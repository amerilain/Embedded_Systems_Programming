#include <stdio.h>
#include <string.h>
#include <ctype.h>
#include "pico/stdlib.h"
#include "uart.h"

#define STRLEN 80
#define UART_NR 0
#define BAUD_RATE 9600
#define UART_TX_PIN 0
#define UART_RX_PIN 1
#define SW_0 9
#define ATTEMPTS 5

bool button_pressed();
void process_dev_eui(const char *response);
void send_at_command();
void read_firmware_version();
void read_dev_eui();

typedef enum {
    WAIT_FOR_BUTTON_PRESS,
    SEND_AT_COMMAND,
    READ_FIRMWARE_VERSION,
    READ_DEVEUI,
    PROCESS_RESPONSE,
    ERROR_STATE
} State;

State current_state = WAIT_FOR_BUTTON_PRESS;

int main() {
    gpio_init(22);
    gpio_set_dir(22, GPIO_OUT);

    gpio_init(SW_0);
    gpio_set_dir(SW_0, GPIO_IN);
    gpio_pull_up(SW_0);

    stdio_init_all();
    uart_setup(UART_NR, UART_TX_PIN, UART_RX_PIN, BAUD_RATE);

    while (true) {
        switch (current_state) {
            case WAIT_FOR_BUTTON_PRESS:
                if (button_pressed()) {
                    printf("Button pressed, starting communication.\n");
                    current_state = SEND_AT_COMMAND;
                }
                break;

            case SEND_AT_COMMAND:
                send_at_command();
                break;

            case READ_FIRMWARE_VERSION:
                read_firmware_version();
                break;

            case READ_DEVEUI:
                read_dev_eui();
                break;

            case PROCESS_RESPONSE:
                // Handled within the read_dev_eui function
                break;

            case ERROR_STATE:
                printf("An error occurred. Going back to waiting for button press.\n");
                current_state = WAIT_FOR_BUTTON_PRESS;
                break;
        }
        sleep_ms(10);
    }

    return 0;
}

bool button_pressed() {
    static uint64_t last_press_time = 0;
    uint64_t current_time = time_us_64();
    if (gpio_get(SW_0) == 0) {
        if (current_time - last_press_time > 500000) { // 500 ms debounce
            last_press_time = current_time;
            return true;
        }
    }
    return false;
}

void process_dev_eui(const char *response) {
    const char *prefix = "+ID: DevEui, ";
    char *start = strstr(response, prefix);
    if (start) {
        start += strlen(prefix);
        char devEui[STRLEN] = {0};
        char *dst = devEui;
        for (char *src = start; *src && *src != '\r'; src++) {
            if (*src != ':') {
                *dst++ = tolower((unsigned char)*src);
            }
        }
        *dst = '\0';
        printf("Processed DevEui: %s\n", devEui);
        current_state = WAIT_FOR_BUTTON_PRESS;
    }
}

void send_at_command() {
    const char at_command[] = "AT\r\n";
    int attempt = 0;
    while (attempt < ATTEMPTS) {
        uart_send(UART_NR, at_command);
        sleep_ms(500); // Wait for response

        char response[STRLEN];
        int len = uart_read(UART_NR, (uint8_t*)response, STRLEN);
        if (len > 0) {
            response[len] = '\0';
            if (strstr(response, "+AT: OK") != NULL) {
                printf("Connected to LoRa module.\n");
                current_state = READ_FIRMWARE_VERSION;
                return;
            }
        }

        attempt++;
    }
    printf("Module not responding.\n");
    current_state = WAIT_FOR_BUTTON_PRESS;
}

void read_firmware_version() {
    const char firmware_cmd[] = "AT+VER\r\n";
    uart_send(UART_NR, firmware_cmd);
    sleep_ms(500); // Wait for response

    char response[STRLEN];
    int len = uart_read(UART_NR, (uint8_t*)response, STRLEN);
    if (len > 0) {
        response[len] = '\0';
        printf("Firmware Version: %s\n", response);
        current_state = READ_DEVEUI;
    } else {
        printf("Module stopped responding.\n");
        current_state = WAIT_FOR_BUTTON_PRESS;
    }
}

void read_dev_eui() {
    const char deveui_cmd[] = "AT+ID=DevEui\r\n";
    uart_send(UART_NR, deveui_cmd);
    sleep_ms(500); // Wait for response

    char response[STRLEN];
    int len = uart_read(UART_NR, (uint8_t*)response, STRLEN);
    if (len > 0) {
        response[len] = '\0';
        current_state = PROCESS_RESPONSE;
        process_dev_eui(response);
    } else {
        printf("Module stopped responding.\n");
        current_state = WAIT_FOR_BUTTON_PRESS;
    }
}
