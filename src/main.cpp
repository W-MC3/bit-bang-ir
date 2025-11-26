#include <avr/io.h>
#include <avr/interrupt.h>
#include <util/delay.h>
#include <stdio.h>
#include <string.h>

#define F_CPU 16000000UL
#define BAUD 9600
#define MYUBRR F_CPU / 16 / BAUD - 1

// ================= UART (Debugging) =================
void uart_init()
{
    UBRR0H = (MYUBRR >> 8);
    UBRR0L = MYUBRR;
    UCSR0B = (1 << TXEN0);
    UCSR0C = (1 << UCSZ01) | (1 << UCSZ00); // 8N1
}

void uart_send_byte(uint8_t b)
{
    while (!(UCSR0A & (1 << UDRE0)))
        ;
    UDR0 = b;
}

void uart_send_string(char *s)
{
    while (*s)
        uart_send_byte(*s++);
}

// ================= 38kHz Carrier (IR Transmitter) =================
void carrier_init()
{
    DDRD |= (1 << PD6);                         // PD6 output
    TCCR0A = (1 << WGM01);                      // CTC mode
    TCCR0B = (1 << CS00);                       // prescaler = 1
    OCR0A = 209;                                // ~38kHz
    TCCR0A &= ~((1 << COM0A1) | (1 << COM0A0)); // Start LED OFF
    PORTD &= ~(1 << PD6);
}

void carrier_on()
{
    TCCR0A |= (1 << COM0A0); // Toggle OC0A on compare match
}

void carrier_off()
{
    TCCR0A &= ~((1 << COM0A1) | (1 << COM0A0)); // Disconnect OC0A
    PORTD &= ~(1 << PD6);                       // Force Low
}

// ================= Software TX (Sending) =================
volatile uint8_t tx_busy = 0;
volatile uint8_t tx_idx = 0;
volatile uint8_t tx_bit_pos = 0;
volatile uint8_t tx_frame[64];
volatile uint8_t tx_len;

void start_send_frame(uint8_t *frame, uint8_t len)
{
    if (len > 64)
        len = 64;
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

    // MSB First
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
            carrier_off(); // Ensure off at end
        }
    }
}

// ================= Software RX (Receiving) =================
#define RX_PIN PD2
#define RX_PORT PIND
#define RX_DDR DDRD

// RX State Machine
#define RX_STATE_SYNC 0
#define RX_STATE_DATA 1

volatile uint8_t rx_state = RX_STATE_SYNC;
volatile uint8_t rx_byte_buffer = 0;
volatile uint8_t rx_bit_count = 0;

// Circular Buffer for received bytes
#define RX_BUF_SIZE 64
volatile uint8_t rx_ring_buffer[RX_BUF_SIZE];
volatile uint8_t rx_head = 0;
volatile uint8_t rx_tail = 0;

void rx_init() { RX_DDR &= ~(1 << RX_PIN); }

void rx_reset_sync()
{
    rx_state = RX_STATE_SYNC;
    rx_byte_buffer = 0;
    rx_bit_count = 0;
}

void rx_push_byte(uint8_t b)
{
    uint8_t next_head = (rx_head + 1) % RX_BUF_SIZE;
    if (next_head != rx_tail)
    { // Prevent overwrite
        rx_ring_buffer[rx_head] = b;
        rx_head = next_head;
    }
}

void rx_next_bit()
{
    // 1. Read Input (INVERTED for IR Receiver: Low = 1, High = 0)
    uint8_t raw_pin = (RX_PORT & (1 << RX_PIN));
    uint8_t bit_val = raw_pin ? 0 : 1;

    // 2. Shift bit into sliding buffer
    rx_byte_buffer = (rx_byte_buffer << 1) | bit_val;

    // 3. State Machine
    if (rx_state == RX_STATE_SYNC)
    {
        // Hunting for Preamble 0x55 (01010101)
        if (rx_byte_buffer == 0x55)
        {
            rx_state = RX_STATE_DATA;
            rx_bit_count = 0;
            rx_push_byte(0x55); // Push the sync byte
            rx_byte_buffer = 0;
        }
    }
    else
    {
        // RX_STATE_DATA: Collect 8 bits
        rx_bit_count++;
        if (rx_bit_count >= 8)
        {
            rx_push_byte(rx_byte_buffer);
            rx_bit_count = 0;
            rx_byte_buffer = 0;
        }
    }
}

// ================= Timer2 1ms Tick =================
void timer2_init()
{
    TCCR2A = (1 << WGM21); // CTC Mode
    TCCR2B = (1 << CS22);  // Prescaler 64
    OCR2A = 249;           // (16MHz / 64 / 1000Hz) - 1 = 249
    TIMSK2 |= (1 << OCIE2A);
}

ISR(TIMER2_COMPA_vect)
{
    if (tx_busy)
        tx_next_bit();

    rx_next_bit();
}

// ================= Protocol =================
uint8_t checksum(uint8_t type, uint8_t len, uint8_t *data)
{
    uint8_t c = type ^ len;
    for (uint8_t i = 0; i < len; i++)
        c ^= data[i];
    return c;
}

// Base function to send any data array
void send_message(uint8_t type, uint8_t *data, uint8_t len)
{
    uint8_t frame[64];
    uint8_t i = 0;

    frame[i++] = 0x55; // Start
    frame[i++] = type;
    frame[i++] = len;
    for (uint8_t j = 0; j < len; j++)
        frame[i++] = data[j];
    frame[i++] = checksum(type, len, data);
    frame[i++] = 0xAA; // End

    start_send_frame(frame, i);
}

// *** NEW HELPER ***
// Calculates length and sends string immediately
void send_string_packet(char *str)
{
    uint8_t len = 0;
    while (str[len] && len < 32)
    {
        len++;
    }
    // Type 0x01 = Text Message
    send_message(0x01, (uint8_t *)str, len);
}

// ================= Decoding (Main Loop) =================
void process_byte(uint8_t b)
{
    static uint8_t state = 0, type, len, cs, payload_idx = 0;
    // Buffer size 33 to allow for a safe null terminator at the end
    static uint8_t payload[33];

    switch (state)
    {
    case 0: // Wait for Sync
        if (b == 0x55)
            state = 1;
        break;

    case 1: // Type
        type = b;
        state = 2;
        break;

    case 2: // Length
        len = b;
        payload_idx = 0;
        if (len > 32)
        {
            state = 0;
            rx_reset_sync();
        }
        else
        {
            state = (len > 0) ? 3 : 4;
        }
        break;

    case 3: // Payload
        payload[payload_idx++] = b;
        if (payload_idx >= len)
        {
            payload[payload_idx] = '\0'; // Null terminate for safe printing
            state = 4;
        }
        break;

    case 4: // Checksum
        cs = b;
        state = 5;
        break;

    case 5: // End Byte
        if (b == 0xAA)
        {
            uint8_t calc_cs = checksum(type, len, payload);
            if (cs == calc_cs)
            {
                uart_send_string("RX Msg: ");
                uart_send_string((char *)payload);
                uart_send_string("\r\n");
            }
            else
            {
                uart_send_string("Err: Checksum\r\n");
            }
        }
        else
        {
            uart_send_string("Err: Frame End\r\n");
        }

        state = 0;
        rx_reset_sync(); // Hunt for next packet
        break;
    }
}

// ================= Main =================
int main()
{
    uart_init();
    carrier_init();
    rx_init();
    timer2_init();
    sei();

    uart_send_string("--- IR Comms Ready ---\r\n");

    uint8_t counter = 0;
    char msg_buffer[32];

    while (1)
    {
        // 1. Transmitter Logic
        if (!tx_busy)
        {
            _delay_ms(1000);

            // Prepare a dynamic string
            sprintf(msg_buffer, "Count: %d", counter++);

            // Send it using the new helper
            send_string_packet(msg_buffer);
        }

        // 2. Receiver Logic
        while (rx_head != rx_tail)
        {
            uint8_t b = rx_ring_buffer[rx_tail];
            rx_tail = (rx_tail + 1) % RX_BUF_SIZE;
            process_byte(b);
        }
    }
}