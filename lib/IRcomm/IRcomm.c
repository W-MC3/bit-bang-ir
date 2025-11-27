#include "IRComm.h"
#include <avr/io.h>
#include <avr/interrupt.h>
#include <string.h>

// Hardcoded for standard Arduino Uno/Nano (ATmega328P)
// Adjust pins if using different hardware
#define RX_PIN PD2
#define RX_PORT PIND
#define RX_DDR DDRD

// ================= GLOBALS =================
volatile uint8_t tx_busy = 0;
volatile uint8_t tx_frame[64];
volatile uint8_t tx_len;
volatile uint8_t tx_idx = 0;
volatile uint8_t tx_bit_pos = 0;

// RX Ring Buffer
#define RX_BUF_SIZE 64
volatile uint8_t rx_ring_buffer[RX_BUF_SIZE];
volatile uint8_t rx_head = 0;
volatile uint8_t rx_tail = 0;

// Received Message Holding Buffer (for the user)
char last_received_msg[IR_MAX_MSG_LEN + 1];
volatile uint8_t msg_available = 0;

// ================= INTERNAL: Carrier =================
void carrier_on() { TCCR0A |= (1 << COM0A0); }
void carrier_off()
{
    TCCR0A &= ~((1 << COM0A1) | (1 << COM0A0));
    PORTD &= ~(1 << PD6);
}

void carrier_init()
{
    DDRD |= (1 << PD6);
    TCCR0A = (1 << WGM01);
    TCCR0B = (1 << CS00);
    OCR0A = 209; // ~38kHz at 16MHz
    carrier_off();
}

// ================= INTERNAL: TX Logic =================
void tx_next_bit()
{
    if (!tx_busy)
        return;
    uint8_t bit = (tx_frame[tx_idx] >> (7 - tx_bit_pos)) & 1;
    if (bit)
        carrier_on();
    else
        carrier_off();

    tx_bit_pos++;
    if (tx_bit_pos >= 8)
    {
        tx_bit_pos = 0;
        tx_idx++;
        if (tx_idx >= tx_len)
        {
            tx_busy = 0;
            carrier_off();
        }
    }
}

// ================= INTERNAL: RX Logic =================
volatile uint8_t rx_state = 0; // 0=Sync, 1=Data
volatile uint8_t rx_byte_buffer = 0;
volatile uint8_t rx_bit_count = 0;

void rx_next_bit()
{
    uint8_t raw_pin = (RX_PORT & (1 << RX_PIN));
    uint8_t bit_val = raw_pin ? 0 : 1; // Active Low
    rx_byte_buffer = (rx_byte_buffer << 1) | bit_val;

    if (rx_state == 0)
    {
        if (rx_byte_buffer == 0x55)
        {
            rx_state = 1;
            rx_bit_count = 0;
            // Add to ring buffer
            uint8_t next_head = (rx_head + 1) % RX_BUF_SIZE;
            if (next_head != rx_tail)
            {
                rx_ring_buffer[rx_head] = 0x55;
                rx_head = next_head;
            }
            rx_byte_buffer = 0;
        }
    }
    else
    {
        rx_bit_count++;
        if (rx_bit_count >= 8)
        {
            uint8_t next_head = (rx_head + 1) % RX_BUF_SIZE;
            if (next_head != rx_tail)
            {
                rx_ring_buffer[rx_head] = rx_byte_buffer;
                rx_head = next_head;
            }
            rx_bit_count = 0;
            rx_byte_buffer = 0;
        }
    }
}

// ================= INTERNAL: Timer =================
ISR(TIMER2_COMPA_vect)
{
    if (tx_busy)
        tx_next_bit();
    rx_next_bit();
}

// ================= PROTOCOL =================
uint8_t checksum(uint8_t type, uint8_t len, uint8_t *data)
{
    uint8_t c = type ^ len;
    for (uint8_t i = 0; i < len; i++)
        c ^= data[i];
    return c;
}

void process_byte(uint8_t b)
{
    static uint8_t state = 0, type, len, cs, payload_idx = 0;
    static uint8_t payload[IR_MAX_MSG_LEN + 1];

    switch (state)
    {
    case 0:
        if (b == 0x55)
            state = 1;
        break;
    case 1:
        type = b;
        state = 2;
        break;
    case 2:
        len = b;
        payload_idx = 0;
        if (len > IR_MAX_MSG_LEN)
        {
            state = 0;
            rx_state = 0;
        } // Reset
        else
            state = (len > 0) ? 3 : 4;
        break;
    case 3:
        payload[payload_idx++] = b;
        if (payload_idx >= len)
        {
            payload[payload_idx] = '\0';
            state = 4;
        }
        break;
    case 4:
        cs = b;
        state = 5;
        break;
    case 5:
        if (b == 0xAA && cs == checksum(type, len, payload))
        {
            // Valid Message Received -> Copy to public buffer
            strcpy(last_received_msg, (char *)payload);
            msg_available = 1;
        }
        state = 0;
        rx_state = 0; // Reset sync
        break;
    }
}

// ================= PUBLIC API =================
void ir_init()
{
    RX_DDR &= ~(1 << RX_PIN); // RX Input
    carrier_init();           // TX Init

    // Timer 2 Init
    TCCR2A = (1 << WGM21);
    TCCR2B = (1 << CS22);
    OCR2A = 249;
    TIMSK2 |= (1 << OCIE2A);
}

void ir_send(char *str)
{
    if (tx_busy)
        return; // Simple collision avoidance

    uint8_t len = 0;
    while (str[len] && len < IR_MAX_MSG_LEN)
        len++;

    uint8_t i = 0;
    tx_frame[i++] = 0x55;
    tx_frame[i++] = 0x01; // Type String
    tx_frame[i++] = len;
    for (uint8_t k = 0; k < len; k++)
        tx_frame[i++] = str[k];
    tx_frame[i++] = checksum(0x01, len, (uint8_t *)str);
    tx_frame[i++] = 0xAA;

    tx_len = i;
    tx_idx = 0;
    tx_bit_pos = 0;
    tx_busy = 1;
}

uint8_t ir_available()
{
    return msg_available;
}

uint8_t ir_read(char *buffer)
{
    if (!msg_available)
        return 0;
    strcpy(buffer, last_received_msg);
    msg_available = 0; // Clear flag
    return strlen(buffer);
}

void ir_update()
{
    // Process ring buffer
    while (rx_head != rx_tail)
    {
        uint8_t b = rx_ring_buffer[rx_tail];
        rx_tail = (rx_tail + 1) % RX_BUF_SIZE;
        process_byte(b);
    }
}