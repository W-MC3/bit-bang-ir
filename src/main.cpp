#include <avr/io.h>
#include <avr/interrupt.h>
#include <util/delay.h>
#include <stdio.h>
#include "IRComm.h"

// Simple UART for debug output to PC
void debug_init()
{
    UBRR0H = 0;
    UBRR0L = 103; // 9600 baud @ 16MHz
    UCSR0B = (1 << TXEN0);
    UCSR0C = (1 << UCSZ01) | (1 << UCSZ00);
}

void debug_print(char *s)
{
    while (*s)
    {
        while (!(UCSR0A & (1 << UDRE0)))
            ;
        UDR0 = *s++;
    }
}

int main()
{
    debug_init();
    ir_init(); // Start the IR library
    sei();     // Enable Interrupts

    debug_print("IR Library Demo\n");

    char msg_out[32];
    char msg_in[32];
    int val1 = 0, val2 = 100;

    while (1)
    {
        // --- SENDING ---
        // Send data every 500ms
        _delay_ms(500);

        sprintf(msg_out, "%d,%d", val1++, val2++);
        ir_send(msg_out);

        // --- RECEIVING ---
        // Must call update frequently to process bits into bytes
        ir_update();

        // Check if we got a full message
        if (ir_available())
        {
            ir_read(msg_in); // Copy message to our buffer

            // Print to PC
            debug_print("Recv: ");
            debug_print(msg_in);
            debug_print("\n");
        }
    }
}