#include <avr/io.h>
#include <avr/interrupt.h>
#include <util/delay.h>

// ====================================================================
// TIMER0 → 38 kHz CARRIER OP PD6 (OC0A)
// ====================================================================
void carrier_init()
{
    DDRD |= (1 << PD6); // PD6 output

    // CTC mode, toggling OC0A
    TCCR0A = (1 << WGM01);
    TCCR0B = (1 << CS00); // prescaler 1
    OCR0A = 209;          // ~38kHz

    // carrier standaard uit
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
// SOFTWARE PROTOCOL: NON-BLOCKING
// ====================================================================
#define BIT_TIME_MS 1 // 1 bit = 1 ms (Timer2)

volatile uint8_t tx_byte, tx_bit_pos;
volatile uint8_t tx_busy = 0;
volatile uint8_t tx_frame[32], tx_len, tx_idx;
volatile uint8_t rx_byte, rx_bit_pos;
volatile uint8_t rx_frame[32], rx_len;
volatile uint8_t rx_ready = 0;

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
// RECEIVER
// ====================================================================
#define RX_PIN 2
#define RX_PORT PIND
#define RX_DDR DDRD

void rx_init() { RX_DDR &= ~(1 << RX_PIN); }

void rx_next_bit()
{
    static uint8_t sample_count = 0;
    static uint8_t bit_val = 0;
    sample_count++;
    if (sample_count >= 1)
    { // sample elke 1ms
        bit_val = (RX_PORT & (1 << RX_PIN)) ? 1 : 0;
        rx_byte <<= 1;
        rx_byte |= bit_val;
        rx_bit_pos++;
        if (rx_bit_pos >= 8)
        {
            rx_frame[rx_len++] = rx_byte;
            rx_bit_pos = 0;
            rx_byte = 0;
            rx_ready = 1;
        }
        sample_count = 0;
    }
}

// ====================================================================
// TIMER2 → 1ms INTERRUPT VOOR BIT TICK
// ====================================================================
void timer2_init()
{
    TCCR2A = (1 << WGM21);   // CTC mode
    TCCR2B = (1 << CS22);    // prescaler 64
    OCR2A = 249;             // 1ms @16MHz
    TIMSK2 |= (1 << OCIE2A); // Compare match interrupt
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
    out[i++] = 0x55;
    out[i++] = type;
    out[i++] = len;
    for (uint8_t j = 0; j < len; j++)
        out[i++] = data[j];
    out[i++] = checksum(type, len, data);
    out[i++] = 0xAA;
    return i;
}

// ====================================================================
// MAIN
// ====================================================================
int main()
{
    carrier_init();
    rx_init();
    timer2_init();
    sei();

    // init frame
    uint8_t txt[] = {'H', 'A', 'L', 'L', 'O'};
    uint8_t frame[16];
    uint8_t flen = build_frame(0x01, txt, 5, frame);

    while (1)
    {
        if (!tx_busy)
            start_send_frame(frame, flen);
        _delay_ms(200);

        if (rx_ready)
        {
            // frame ontvangen
            rx_ready = 0;
            // hier kan je checksums en start/stop bytes checken
            // en eventueel een ACK terugsturen
        }
    }
}
