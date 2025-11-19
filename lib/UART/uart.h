#ifndef UART_H
#define UART_H

#include <avr/io.h>

void uart_init(unsigned long baud);
void uart_send_char(char c);
void uart_send_string(const char *str);

#endif