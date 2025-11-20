/****************************************************************************************
* File:         print.c
* Author:       Michiel Dirks
* Created on:   16-09-2025
* Company:      Windesheim
* Website:      https://www.windesheim.nl/opleidingen/voltijd/bachelor/ict-zwolle
****************************************************************************************/

#include <stdarg.h>
#include <stdio.h>
#include <stdint.h>
#include <stdbool.h>
#include "print.h"

#include <avr/common.h>
#include <avr/interrupt.h>

#define PRINT_BUFFER_SIZE 64
#define LINE_BUFFER_SIZE 32

static char lineBuffer[LINE_BUFFER_SIZE];
static uint8_t lineIndex = 0;

static char printBuffer[PRINT_BUFFER_SIZE];

static void (*uart_send_func)(void*, uint8_t) = NULL;
static bool (*uart_available_func)(void) = NULL;
static uint8_t (*uart_read_func)(void) = NULL;

/**
 * @brief Initializes UART-backed printing helpers.
 */
void print_init(
    void (*sendFunc)(void*, uint8_t),
    bool (*availableFunc)(void),
    uint8_t (*readFunc)(void)
) {
    uart_send_func = sendFunc;
    uart_available_func = availableFunc;
    uart_read_func = readFunc;
}

/**
 * @brief Formatted print to the configured UART output.
 */
void print(const char *fmt, ...) {
    if (!uart_send_func) return;

    va_list args;
    va_start(args, fmt);
    int len = vsnprintf(printBuffer, PRINT_BUFFER_SIZE, fmt, args);
    va_end(args);

    if (len > 0) {
        if (len > PRINT_BUFFER_SIZE) len = PRINT_BUFFER_SIZE;
        uart_send_func(printBuffer, len);
    }
}

/**
 * @brief Checks whether a byte can be read from the input stream.
 */
bool byte_available(void) {
    return uart_available_func ? uart_available_func() : false;
}

/**
 * @brief Reads a single byte from the input stream if available.
 */
uint8_t read_byte(void) {
    return uart_read_func ? uart_read_func() : 0;
}

/**
 * @brief Reads a line terminated by a newline character into outLine.
 */
bool read_line(char *outLine, uint8_t maxLen) {
    while (byte_available()) {
        char c = read_byte();
        if (c == '\r') continue;
        if (c == '\n') {
            if (lineIndex < maxLen) {
                lineBuffer[lineIndex] = '\0';
                for (uint8_t i = 0; i <= lineIndex && i < maxLen; i++) {
                    outLine[i] = lineBuffer[i];
                }
            }
            lineIndex = 0;
            return true;
        }
        if (lineIndex < LINE_BUFFER_SIZE - 1) {
            lineBuffer[lineIndex++] = c;
        }
    }
    return false;
}
