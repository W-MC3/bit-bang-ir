#include "uart.h"
#include <util/delay.h>

void uart_init(unsigned long baud)
{
    unsigned int ubrr = (F_CPU / 16 / baud) - 1;
    UBRR0H = (unsigned char)(ubrr >> 8);
    UBRR0L = (unsigned char)ubrr;

    UCSR0B = (1 << TXEN0);                  // Transmitter aan
    UCSR0C = (1 << UCSZ01) | (1 << UCSZ00); // 8N1
}

void uart_send_char(char c)
{
    while (!(UCSR0A & (1 << UDRE0)))
        ; // wachten tot buffer leeg
    UDR0 = c;
}

void uart_send_string(const char *str)
{
    while (*str)
    {
        uart_send_char(*str++);
    }
}
