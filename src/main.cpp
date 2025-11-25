#include <avr/io.h>
#include <avr/interrupt.h>
#include <util/delay.h>

#define F_CPU 16000000UL
#define BAUD 9600
#define MYUBRR F_CPU / 16 / BAUD - 1

// UART INIT
void uart_init()
{
    UBRR0H = (MYUBRR >> 8);
    UBRR0L = MYUBRR;
    UCSR0B = (1 << TXEN0);                  // TX aan
    UCSR0C = (1 << UCSZ01) | (1 << UCSZ00); // 8N1
}

void uart_send_byte(uint8_t b)
{
    while (!(UCSR0A & (1 << UDRE0)))
        ;
    UDR0 = b;
}

void uart_send_string(uint8_t *s, uint8_t len)
{
    for (uint8_t i = 0; i < len; i++)
        uart_send_byte(s[i]);
}

// ====================================================================
// TIMER0 → 38 kHz CARRIER OP PD6
// ====================================================================
void carrier_init()
{
    DDRD |= (1 << PD6);    // output
    TCCR0A = (1 << WGM01); // CTC
    TCCR0B = (1 << CS00);  // prescaler 1
    OCR0A = 209;           // ~38kHz
    TCCR0A &= ~((1 << COM0A1) | (1 << COM0A0));
    PORTD &= ~(1 << PD6);
}

void carrier_on() { TCCR0A |= (1 << COM0A0); }
void carrier_off()
{
    TCCR0A &= ~((1 << COM0A1) | (1 << COM0A0));
    PORTD &= ~(1 << PD6);
}

// ====================================================================
// SOFTWARE BIT-LEVEL PROTOCOL
// ====================================================================
volatile uint8_t tx_busy = 0, tx_idx = 0, tx_bit_pos = 0;
volatile uint8_t tx_frame[32], tx_len;
volatile uint8_t rx_byte = 0, rx_bit_pos = 0;
volatile uint8_t rx_frame[32], rx_len = 0, rx_ready = 0;

void start_send_frame(uint8_t *frame, uint8_t len)
{
    for (uint8_t i = 0; i < len; i++)
        tx_frame[i] = frame[i];
    tx_len = len;
    tx_idx = 0;
    tx_bit_pos = 0;
    tx_busy = 1;
}

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

// ====================================================================
// RECEIVER PIN
// ====================================================================
#define RX_PIN 2
#define RX_PORT PIND
#define RX_DDR DDRD

// ====================================================================
// Hier gaat fout
// ====================================================================
void rx_init() { RX_DDR &= ~(1 << RX_PIN); }

void rx_next_bit()
{
    static uint8_t sample_count = 0;
    static uint8_t bit_val = 0;
    sample_count++;
    if (sample_count >= 1)
    { // elke 1ms
        bit_val = (RX_PORT & (1 << RX_PIN)) ? 1 : 0;
        rx_byte <<= 1;
        rx_byte |= bit_val;
        rx_bit_pos++;
        if (rx_bit_pos >= 8)
        {
            rx_frame[rx_len++] = rx_byte;
            rx_byte = 0;
            rx_bit_pos = 0;
            rx_ready = 1;
        }
        sample_count = 0;
    }
}

// ====================================================================
// TIMER2 → 1ms TICK
// ====================================================================
void timer2_init()
{
    TCCR2A = (1 << WGM21);
    TCCR2B = (1 << CS22);
    OCR2A = 249;
    TIMSK2 |= (1 << OCIE2A);
}

ISR(TIMER2_COMPA_vect)
{
    if (tx_busy)
        tx_next_bit();
    rx_next_bit();
}

// ====================================================================
// FRAME BUILD & CHECKSUM
// ====================================================================
uint8_t checksum(uint8_t type, uint8_t len, uint8_t *data)
{
    uint8_t c = type ^ len;
    for (uint8_t i = 0; i < len; i++)
        c ^= data[i];
    return c;
}

uint8_t build_frame(uint8_t type, uint8_t *data, uint8_t len, uint8_t *out)
{
    uint8_t i = 0;
    out[i++] = 0x55; // start
    out[i++] = type;
    out[i++] = len;
    for (uint8_t j = 0; j < len; j++)
        out[i++] = data[j];
    out[i++] = checksum(type, len, data);
    out[i++] = 0xAA; // stop
    return i;
}

// ====================================================================
// FRAME DECODER
// ====================================================================
void decode_byte(uint8_t b)
{

    static uint8_t state = 0, type, len, cs, payload[32], payload_idx = 0;

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
        state = (len > 0) ? 3 : 4;
        break;
    case 3:
        payload[payload_idx++] = b;
        if (payload_idx >= len)
            state = 4;
        break;
    case 4:
        cs = b;
        state = 5;
        break;
    case 5:
        if (b == 0xAA && cs == checksum(type, len, payload))
            uart_send_string(payload, len);
        else
            uart_send_string((uint8_t *)"ERR", 3); // Debug: indicate frame error
        state = 0;
        break;
    }
}

// ====================================================================
// MAIN
// ====================================================================
int main()
{
    uart_init();
    carrier_init();
    rx_init();
    timer2_init();
    sei();

    uint8_t txt[] = {'H', 'A', 'L', 'L', 'O', '\n'};
    uint8_t frame[16];
    uint8_t flen = build_frame(0x01, txt, 6, frame);

    while (1)
    {
        // if (!tx_busy)
        //     start_send_frame(frame, flen);
        // _delay_ms(200);

        if (rx_ready)
        {
            rx_ready = 0;
            for (uint8_t i = 0; i < rx_len; i++)
                decode_byte(rx_frame[i]);
            rx_len = 0;
        }
    }
}
