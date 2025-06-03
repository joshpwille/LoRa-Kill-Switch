// This script checks uses the esp to communicate with the SIM700G to:
//  1. AT+CGNSPWR=1 → turn on the GNSS (GPS) power
//  2. AT+CGNSINF → return current GPS status and location data (if available)

#include <stdio.h>
#include <string.h>
#include "driver/uart.h"
#include "driver/gpio.h"
#include "freertos/FreeRTOS.h"
#include "freertos/task.h"

#define UART_PORT UART_NUM_2
#define BUF_SIZE (1024)

// SIM7000G Control Pins
#define TXD_PIN 27         // ESP32 TX -> SIM7000G RX
#define RXD_PIN 26         // ESP32 RX <- SIM7000G TX
#define PWRKEY_PIN 4       // Controls SIM7000G PWRKEY

void power_on_sim7000g() {
    // Initialize PWRKEY pin
    esp_rom_gpio_pad_select_gpio(PWRKEY_PIN);
    gpio_set_direction(PWRKEY_PIN, GPIO_MODE_OUTPUT);

    // Toggle PWRKEY LOW for >1s to power on SIM7000G
    gpio_set_level(PWRKEY_PIN, 0);
    vTaskDelay(pdMS_TO_TICKS(1500));  // Hold LOW for 1.5 seconds
    gpio_set_level(PWRKEY_PIN, 1);
    vTaskDelay(pdMS_TO_TICKS(100));   // Let it settle
}

void app_main() {
    // Power on the SIM7000G module
    printf("Booting: powering on SIM7000G and initializing UART...\n");
    power_on_sim7000g();

    // Configure UART
    const uart_config_t uart_config = {
        .baud_rate = 9600,  // SIM7000G default
        .data_bits = UART_DATA_8_BITS,
        .parity    = UART_PARITY_DISABLE,
        .stop_bits = UART_STOP_BITS_1,
        .flow_ctrl = UART_HW_FLOWCTRL_DISABLE
    };

    uart_param_config(UART_PORT, &uart_config);
    uart_set_pin(UART_PORT, TXD_PIN, RXD_PIN, UART_PIN_NO_CHANGE, UART_PIN_NO_CHANGE);
    uart_driver_install(UART_PORT, BUF_SIZE * 2, 0, 0, NULL, 0);

// Reset GPS power to force clean start
    uart_write_bytes(UART_PORT, "AT+CGNSPWR=0\r\n", strlen("AT+CGNSPWR=0\r\n"));
    vTaskDelay(pdMS_TO_TICKS(2000));  // wait 2 seconds
    uart_write_bytes(UART_PORT, "AT+CGNSPWR=1\r\n", strlen("AT+CGNSPWR=1\r\n"));
    vTaskDelay(pdMS_TO_TICKS(2000));  // wait for GPS to power back up

// Optional: set NMEA sentence output type (for better fix)
    uart_write_bytes(UART_PORT, "AT+CGNSSEQ=\"RMC\"\r\n", strlen("AT+CGNSSEQ=\"RMC\"\r\n"));
    vTaskDelay(pdMS_TO_TICKS(1000));

    // Send AT command
    const char *cmd = "AT\r\n";
    uart_write_bytes(UART_PORT, cmd, strlen(cmd));
    printf("Sent command: %s", cmd);
    // Enable GPS
    uart_write_bytes(UART_PORT, "AT+CGNSPWR=1\r\n", strlen("AT+CGNSPWR=1\r\n"));
    vTaskDelay(pdMS_TO_TICKS(2000));  // wait for GPS to power up
    // Request GPS info
    uart_write_bytes(UART_PORT, "AT+CGNSINF\r\n", strlen("AT+CGNSINF\r\n"));

    // Read response loop
    uint8_t data[BUF_SIZE];
    while (1) {
        int len = uart_read_bytes(UART_PORT, data, BUF_SIZE - 1, 1000 / portTICK_PERIOD_MS);
        if (len > 0) {
            data[len] = '\0';  // Null-terminate for string display
            printf("Received (%d bytes): %s\n", len, (char *)data);
        }
        uart_write_bytes(UART_PORT, "AT+CGNSINF\r\n", strlen("AT+CGNSINF\r\n"));
        vTaskDelay(pdMS_TO_TICKS(3000));
    }
}
