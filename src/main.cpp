#include <avr/io.h>
#include <avr/interrupt.h>
#include <util/delay.h>

// ====================================================================
//  TIMER0 → 38 kHz CARRIER ON PD6 (OC0A)
// ====================================================================
void carrier_init()
{
    DDRD |= (1 << PD6); // PD6 = output

    // Timer0 CTC mode
    TCCR0A = (1 << WGM01);
    TCCR0B = (1 << CS00); // prescaler 1
    OCR0A = 209;          // ~38 kHz

    // Carrier standaard uit
    TCCR0A &= ~((1 << COM0A1) | (1 << COM0A0));
    PORTD &= ~(1 << PD6); // Pin laag maken
}

void carrier_on()
{
    // Zorg dat OC0A toggelt
    TCCR0A = (TCCR0A & ~(1 << COM0A1)) | (1 << COM0A0);
}

void carrier_off()
{
    // OC0A disconnect
    TCCR0A &= ~((1 << COM0A1) | (1 << COM0A0));
    // Zet pin expliciet laag
    PORTD &= ~(1 << PD6);
}

// ====================================================================
//  BIT EN BYTE ZENDEN (OOK VOOR FRAMES)
// ====================================================================
void send_bit(uint8_t bit)
{
    if (bit)
        carrier_on();
    else
        carrier_off();

    for (volatile uint16_t i = 0; i < 1200; i++)
        ;          // 600 µs
    carrier_off(); // altijd afsluiten
}

void send_byte(uint8_t b)
{
    for (int8_t i = 7; i >= 0; i--)
        send_bit((b >> i) & 1);
}

// ====================================================================
//  FRAME PROTOCOL
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

void send_frame(uint8_t *frame, uint8_t len)
{
    carrier_off();
    for (volatile uint32_t w = 0; w < 10000; w++)
        ;

    for (uint8_t i = 0; i < len; i++)
        send_byte(frame[i]);

    carrier_off();
}

// ====================================================================
//  SIMPLE SOFTWARE RECEIVER (PD2)
// ====================================================================
#define RX_BIT 2

void rx_init()
{
    DDRD &= ~(1 << RX_BIT);
}

uint8_t read_bit()
{
    uint8_t high = 0;

    for (uint8_t i = 0; i < 50; i++)
    {
        if (PIND & (1 << RX_BIT))
            high++;
        for (volatile uint8_t d = 0; d < 20; d++)
            ;
    }

    return (high > 25);
}

uint8_t read_byte()
{
    uint8_t b = 0;
    for (uint8_t i = 0; i < 8; i++)
    {
        b <<= 1;
        b |= read_bit();
    }
    return b;
}

// ====================================================================
//  DUPLEXING MET TIMER2 (1ms INTERRUPT)
// ====================================================================
volatile uint8_t has_token = 1; // 1 = we mogen zenden
volatile uint16_t slot_counter = 0;

// Timer2 → CTC mode → 1ms interrupt
void duplex_timer_init()
{
    TCCR2A = (1 << WGM21);   // CTC
    TCCR2B = (1 << CS22);    // Prescaler 64
    OCR2A = 249;             // 1ms
    TIMSK2 |= (1 << OCIE2A); // enable compare interrupt
}

ISR(TIMER2_COMPA_vect)
{
    slot_counter++;

    if (slot_counter >= 100) // elke 100 ms token wisselen
    {
        slot_counter = 0;
        has_token = !has_token;
    }
}

// ====================================================================
//  SEND & RECEIVE LOOPS
// ====================================================================
void send_loop()
{
    uint8_t txt[] = {'H', 'A', 'L', 'L', 'O'};
    uint8_t frame[16];
    uint8_t len = build_frame(0x01, txt, 5, frame);

    send_frame(frame, len);
}

void receive_loop()
{
    uint8_t b;

    do
    {
        b = read_byte();
    } while (b != 0x55);

    uint8_t type = read_byte();
    uint8_t len = read_byte();

    uint8_t payload[32];
    for (uint8_t i = 0; i < len; i++)
        payload[i] = read_byte();

    uint8_t cs = read_byte();
    uint8_t stop = read_byte();

    if (stop != 0xAA)
        return;
    if (cs != checksum(type, len, payload))
        return;

    // ACK terugsturen
    uint8_t resp[8];
    uint8_t rl = build_frame(0xFE, 0, 0, resp);
    send_frame(resp, rl);
}

// ====================================================================
//  MAIN
// ====================================================================
int main()
{
    carrier_init();
    rx_init();
    duplex_timer_init();

    sei();

    while (1)
    {
        if (has_token)
            send_loop();
        else
            receive_loop();
    }
}
