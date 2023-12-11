#include <stdio.h>
#include <stdlib.h>
#include <string.h>
#include "pico/stdlib.h"
#include "hardware/i2c.h"

static int toStart = 0;

#define IN1 2
#define IN2 3
#define IN3 6
#define IN4 13
#define ADC_1 28

void init();
int calibrate();
int run(int toStep);
void trimString(char *str);

        static const uint8_t clockwise[8][4] = {
        {1,0,0,0},
        {1,1,0,0},
        {0,1,0,0},
        {0,1,1,0},
        {0,0,1,0},
        {0,0,1,1},
        {0,0,0,1},
        {1,0,0,1}
};

uint8_t counterClockwise[8][4] = {
        {1,0,0,1},
        {0,0,0,1},
        {0,0,1,1},
        {0,0,1,0},
        {0,1,1,0},
        {0,1,0,0},
        {1,1,0,0},
        {1,0,0,0}
};



int main() {
    stdio_init_all();
    init();
    sleep_ms(2000);
    printf("Device is on..\n");

    int steps = 0;
    int step = 0;
    int currentStep = 0;
    int toStep = 0;
    char command[50];
    int commandIndex = 0;
    bool calibarated = false;

    while(1){
        while(uart_is_readable(uart0)){
            char c = uart_getc(uart0);
            if(c == '\n' || c == '\r'){
                command[commandIndex] = '\0';
                trimString(command);
                printf("Processed Command: [%s]\n", command);

                if(strcmp(command, "status") == 0){
                    printf("Status Command Processed. Calibrated: %s, Steps per Revolution: %d\n", calibarated ? "Yes" : "No", steps);
                }
                else if(strcmp(command, "calib") == 0){
                    printf("Calibration Command Processed\n");
                    steps = calibrate()/3;
                    calibarated = true;
                    step = steps / 8;
                    printf("Calibration Complete. Steps per Revolution: %d, Step: %d\n", steps, step);
                }
                else if(strncmp(command, "run", 3) == 0){
                    printf("Run Command Processed\n");
                    if(calibarated) {
                        int runSteps;
                        if(strlen(command) > 3) {
                            runSteps = atoi(command + 4) * (steps / 8);
                        } else {
                            runSteps = steps;
                        }
                        printf("Running motor for %d steps.\n", runSteps);
                        currentStep = run(runSteps);
                        printf("Run Completed. Current Step: %d\n", currentStep);
                    } else {
                        printf("Motor not calibrated. Please calibrate first.\n");
                    }
                }

                memset(command, 0, sizeof(command));
                commandIndex = 0;
            }
            else if(c >= 32 && c < 127){
                command[commandIndex++] = c;
            }
        }
    }

    return 0;
}


void init(){
    gpio_init(IN1);
    gpio_init(IN2);
    gpio_init(IN3);
    gpio_init(IN4);
    gpio_init(ADC_1);
    gpio_set_dir(IN1, GPIO_OUT);
    gpio_set_dir(IN2, GPIO_OUT);
    gpio_set_dir(IN3, GPIO_OUT);
    gpio_set_dir(IN4, GPIO_OUT);
    gpio_set_dir(ADC_1, GPIO_IN);
    gpio_pull_up(ADC_1);

}

int calibrate(){
    int stepCount = 0;
    while(gpio_get(ADC_1) == 1){
        for (int i = toStart; i < 8; i++) {
            gpio_put(IN1, clockwise[i][0]);
            gpio_put(IN2, clockwise[i][1]);
            gpio_put(IN3, clockwise[i][2]);
            gpio_put(IN4, clockwise[i][3]);
            sleep_ms(1);
            toStart++;
            if (toStart == 8){
                toStart = 0;
            }
            if(gpio_get(ADC_1) == 0){
                break;
            }
        }
    }
    printf("Calibaration start\n");
    int count = 0;
    while (count < 3){
        while (gpio_get(ADC_1) == 0) {
            for (int i = toStart; i < 8; i++) {
                gpio_put(IN1, clockwise[i][0]);
                gpio_put(IN2, clockwise[i][1]);
                gpio_put(IN3, clockwise[i][2]);
                gpio_put(IN4, clockwise[i][3]);
                sleep_ms(1);
                stepCount++;
                toStart++;
                if (toStart == 8){
                    toStart = 0;
                }
                if(gpio_get(ADC_1) == 1){
                    break;
                }
            }
        }
        while (gpio_get(ADC_1) == 1) {
            for (int i = toStart; i < 8; i++) {
                gpio_put(IN1, clockwise[i][0]);
                gpio_put(IN2, clockwise[i][1]);
                gpio_put(IN3, clockwise[i][2]);
                gpio_put(IN4, clockwise[i][3]);
                sleep_ms(1);
                stepCount++;
                toStart++;
                if (toStart == 8){
                    toStart = 0;
                }
                if(gpio_get(ADC_1) == 0){
                    break;
                }
            }
        }
        count++;
    }
    return stepCount;
}


int run(int toStep){
    int stepCount = 0;
    while (stepCount < toStep){
        for (int i = toStart; i < 8 && stepCount < toStep; i++) { // Ensure stepCount doesn't exceed toStep
            gpio_put(IN1, clockwise[i][0]);
            gpio_put(IN2, clockwise[i][1]);
            gpio_put(IN3, clockwise[i][2]);
            gpio_put(IN4, clockwise[i][3]);
            sleep_ms(1);
            stepCount++;
            toStart++;
            if (toStart == 8){
                toStart = 0;
            }
        }
    }
    return stepCount;
}


void trimString(char *str) {
    int length = strlen(str);
    while (length > 0 && (str[length-1] == '\n' || str[length-1] == '\r')) {
        str[--length] = '\0';
    }
}