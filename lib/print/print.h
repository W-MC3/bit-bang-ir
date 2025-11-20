/****************************************************************************************
* File:         print.h
* Author:       Michiel Dirks
* Created on:   16-09-2025
* Company:      Windesheim
* Website:      https://www.windesheim.nl/opleidingen/voltijd/bachelor/ict-zwolle
****************************************************************************************/

#ifndef CODE_PRINT_H
#define CODE_PRINT_H

#include <stdint.h>

/**
 * @brief Initializes the printing utility with UART callbacks.
 * @param sendFunc Function to send a buffer over UART.
 * @param availableFunc Function returning true if a byte is available to read.
 * @param readFunc Function to read a single incoming byte.
 */
void print_init(
    void (*sendFunc)(void*, uint8_t),
    bool (*availableFunc)(void),
    uint8_t (*readFunc)(void)
);

/**
 * @brief Formatted print to UART (printf-like).
 * @param fmt Format string.
 * @param ... Variadic arguments.
 */
void print(const char *fmt, ...);

/**
 * @brief Checks if at least one byte is available from the input stream.
 */
bool byte_available(void);

/**
 * @brief Reads a single byte from the input stream.
 * @return The byte read, or 0 if none available.
 */
uint8_t read_byte(void);

/**
 * @brief Reads a line terminated by '\n' from the input stream.
 * @param outLine Destination buffer.
 * @param maxLen Maximum bytes to copy including terminator.
 * @return true if a full line was read, else false.
 */
bool read_line(char *outLine, uint8_t maxLen);


#endif //CODE_PRINT_H
