#define F_CPU 16000000UL
#include <avr/io.h>
#include <avr/interrupt.h>
#include <util/delay.h>
#include "uart.h"

#define IR_LED_PIN PB3     // D3 → IR LED output
#define IR_RECEIVE_PIN PD2 // D2 → IR receiver (INT0)

#define MAX_BUFFER 32

#define START_BYTE 0xFF
#define STOP_BYTE 0xAA
#define ACK_RESP 0xCC

volatile uint8_t rx_buffer[MAX_BUFFER]; // ontvangen bytes buffer
volatile uint8_t rx_index = 0;          // index in buffer
volatile uint8_t bit_index = 0;         // bit teller
volatile uint8_t received_byte = 0;     // byte in opbouw

volatile uint8_t transmitting = 0; // zendstatus
volatile uint8_t waiting_ack = 0;  // wacht op ACK
volatile uint8_t can_send = 1;     // 1 = lijn vrij

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
    }
    else
    {
        carrier_on(300); // '0'
    }
    _delay_us(600);
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
void send_string_with_ack(const char *str)
{
    if (!can_send)
        return; // lijn bezet

    send_byte(START_BYTE);
    for (; *str; str++)
    {
        send_byte((uint8_t)(*str));
        _delay_ms(20); // kleine pauze tussen bytes
    }
    send_byte(STOP_BYTE);

    waiting_ack = 1;
    can_send = 0;
}

void send_ack_response(void)
{
    send_byte(ACK_RESP);
}

//------------------- Processing_rx_buffer-------------------//
void process_rx_buffer(void)
{
    if (rx_index == 0)
        return;

    if (rx_buffer[0] == START_BYTE)
    { // startbyte
        // Print string tot ACK
        for (uint8_t i = 1; i < rx_index; i++)
        {
            if (rx_buffer[i] == STOP_BYTE)
                break;                    // einde van de string
            uart_send_char(rx_buffer[i]); // stuur karakter naar UART
        }
        uart_send_char('\n'); // optioneel: newline

        if (!transmitting)
        {
            send_ack_response();
            _delay_ms(10);
            can_send = 1;
        }
    }
    rx_index = 0; // buffer reset
}

//------------------- Ontvangen via INT0 -------------------//
ISR(INT0_vect)
{
    if (transmitting)
        return;

    static uint32_t last_time = 0;
    uint32_t now = TCNT1;
    uint32_t pulse_width = now - last_time;
    last_time = now;

    if (pulse_width > 500)
    {
        received_byte |= (1 << bit_index);
    }
    else
    {
        received_byte &= ~(1 << bit_index);
    }
    bit_index++;

    if (bit_index >= 8)
    {
        if (rx_index < MAX_BUFFER)
        {
            rx_buffer[rx_index++] = received_byte;
        }

        if (received_byte == START_BYTE)
        {
            can_send = 0;
        }

        else if (received_byte == STOP_BYTE)
        {
        }
        else if (received_byte == ACK_RESP)
        {
            if (waiting_ack)
            {
                can_send = 1;
                waiting_ack = 0;
            }
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

    _delay_ms(200);

    while (1)
    {
        if (can_send && !waiting_ack)
        {
            send_string_with_ack("Hello World!");
        }
        process_rx_buffer();
        _delay_ms(50);
    }
}