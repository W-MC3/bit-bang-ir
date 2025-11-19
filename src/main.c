#define F_CPU 16000000UL
#include <avr/io.h>
#include <avr/interrupt.h>
#include <util/delay.h>
#include "uart.h"

#define IR_LED_PIN PB3     // D3 → IR LED output
#define IR_RECEIVE_PIN PD2 // D2 → IR receiver (INT0)

#define MAX_BUFFER 32

volatile uint8_t rx_buffer[MAX_BUFFER]; // ontvangen bytes buffer
volatile uint8_t rx_index = 0;          // index in buffer
volatile uint8_t bit_index = 0;         // bit teller
volatile uint8_t received_byte = 0;     // byte in opbouw

volatile uint8_t transmitting = 0; // zendstatus

//------------------- Carrier signal -------------------//
void carrier_on(uint16_t duration_us)
{
    uint16_t cycles = duration_us / 26; // 26us periode ~ 38 kHz
    for (uint16_t i = 0; i < cycles; i++)
    {
        PORTB |= (1 << IR_LED_PIN);
        _delay_us(13);
        PORTB &= ~(1 << IR_LED_PIN);
        _delay_us(13);
    }
}

//------------------- Bit & Byte verzenden -------------------//
void send_bit(uint8_t bit)
{
    if (bit)
    {
        carrier_on(600); // '1'
        _delay_us(600);
    }
    else
    {
        carrier_on(300); // '0'
        _delay_us(600);
    }
}

void send_byte(uint8_t byte)
{
    transmitting = 1;
    for (uint8_t i = 0; i < 8; i++)
    {
        send_bit((byte >> i) & 0x01); // LSB eerst
    }
    transmitting = 0;
}

//------------------- String verzenden -------------------//
void send_string(const char *str)
{
    send_byte(0xFF); // Start byte
    for (; *str; str++)
    {
        send_byte(*str);
        _delay_ms(50); // korte pauze tussen bytes
    }
    send_byte(0xAA); // Einde byte
}

//------------------- Processing_rx_buffer-------------------//
void process_rx_buffer(void)
{
    if (rx_index == 0)
        return;

    if (rx_buffer[0] == 0xFF)
    { // startbyte
        // Print string tot ACK
        for (uint8_t i = 1; i < rx_index; i++)
        {
            if (rx_buffer[i] == 0xAA)
                break;                    // einde van de string
            uart_send_char(rx_buffer[i]); // stuur karakter naar UART
        }
        uart_send_char('\n'); // optioneel: newline
    }
    rx_index = 0; // buffer reset
}

//------------------- Ontvangen via INT0 -------------------//
ISR(INT0_vect)
{
    static uint32_t last_time = 0;
    uint32_t now = TCNT1;
    uint32_t pulse_width = now - last_time;
    last_time = now;

    if (pulse_width > 500)
    { // filter voor '1'
        received_byte |= (1 << bit_index);
    }

    bit_index++;
    if (bit_index >= 8)
    {
        // volledige byte ontvangen
        if (rx_index < MAX_BUFFER)
        {
            rx_buffer[rx_index++] = received_byte;
        }
        received_byte = 0;
        bit_index = 0;
    }
}

//------------------- Main -------------------//
int main(void)
{

    DDRB |= (1 << IR_LED_PIN) | (1 << PB0); // PB0 als test output
    PORTB &= ~(1 << IR_LED_PIN);

    // int0 setup
    EICRA |= (1 << ISC00); // Rising edge
    EIMSK |= (1 << INT0);  // Enable INT0

    // Timer1 setup
    TCCR1B |= (1 << WGM12) | (1 << CS10);
    TCNT1 = 0;

    uart_init(9600);
    sei(); // Enable global interrupts

    while (1)
    {
        // Voorbeeld: zend een string elke 5 seconden
        send_string("Hello World");
        _delay_ms(5000);

        // Verwerk ontvangen data
        process_rx_buffer();
    }
}