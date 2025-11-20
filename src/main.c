// main.cpp
// ATmega328P bare-minimum glue + allowed libs: SPI, Adafruit_ILI9341, nunchuk.h, uart.h
// ROLE = 1 -> controls Blue ball (sends CMD_BLUE_POS)
// ROLE = 2 -> controls Red ball  (sends CMD_RED_POS)
#define ROLE 1

#include <avr/io.h>
#include <avr/interrupt.h>
#include <util/delay.h>

#include <SPI.h>
#include <Adafruit_GFX.h>
#include <Adafruit_ILI9341.h>

#include "nunchuk.h" // provided by you (nunchuk_begin(), nunchuk_get_state(), and state struct)
#include "uart.h"    // your UART lib (uart_init, uart_send_string, etc.)

// ==== Display pins (change if needed) ====
#define TFT_CS 10
#define TFT_DC 9
#define TFT_RST 8

Adafruit_ILI9341 tft = Adafruit_ILI9341(TFT_CS, TFT_DC, TFT_RST);

// ==== Screen & ball settings ====
const uint16_t SCREEN_W = 320;
const uint16_t SCREEN_H = 240;
const uint8_t BALL_RADIUS = 8; // user choice B

// ==== Protocol commands ====
#define CMD_BLUE_POS 0x20
#define CMD_RED_POS 0x21
#define CMD_FRAME_SECOND_FLAG 0x80

// ==== NEC timings (microseconds) ====
const uint16_t NEC_LEADER_MARK = 9000;
const uint16_t NEC_LEADER_SPACE = 4500;
const uint16_t NEC_BIT_MARK = 560;
const uint16_t NEC_ONE_SPACE = 1690;
const uint16_t NEC_ZERO_SPACE = 560;
const uint16_t NEC_END_MARK = 560;

// ==== IO pins for IR ====
const uint8_t IR_TX_PIN = 3; // OC2A (D3) - hardware toggle used
const uint8_t IR_RX_PIN = 2; // INT0 (D2) - IR receiver output

// ==== rate limiting for sends (ms) ====
const uint16_t SEND_INTERVAL_MS = 80;
static volatile uint32_t g_millis = 0;
static uint32_t lastSendMs = 0;

// ==== positions (volatile because ISR / other code may update) ====
volatile uint8_t blue_x = SCREEN_W / 4;
volatile uint8_t blue_y = SCREEN_H / 2;
volatile uint8_t red_x = SCREEN_W * 3 / 4;
volatile uint8_t red_y = SCREEN_H / 2;

// ==== pulse buffer for edges from INT0 ====
#define PULSE_BUF_SIZE 256
volatile uint32_t pulseBuf[PULSE_BUF_SIZE];
volatile uint8_t pulseHead = 0;
volatile uint8_t pulseTail = 0;

// ==== two-frame state ====
volatile bool have_first_frame = false;
volatile uint8_t pending_cmd = 0;
volatile uint8_t pending_x = 0;

// Forward declarations
void spi_setup_and_tft_init(void);
void timer0_init_millis(void);
uint32_t millis_custom(void);
void timer1_init_for_micro(void);
void timer2_init_carrier(void);
void nec_mark_us(uint16_t us);
void nec_space_us(uint16_t us);
void nec_send32(uint32_t data);
void send_payload_cmd(uint8_t cmd, uint8_t x, uint8_t y);
bool tryDecodeNECFrame(uint32_t *outFrame);
void processFrame(uint32_t frame);
void drawScene(void);

// -------------------- utilities --------------------
static inline int32_t map_int32(int32_t x, int32_t in_min, int32_t in_max, int32_t out_min, int32_t out_max)
{
    // integer safe map
    if (x <= in_min)
        return out_min;
    if (x >= in_max)
        return out_max;
    return (int32_t)((x - in_min) * (out_max - out_min) / (in_max - in_min) + out_min);
}

// -------------------- millis (Timer0 overflow) --------------------
// We'll configure Timer0 to overflow every 1 ms (approx) using prescaler 64
ISR(TIMER0_OVF_vect)
{
    // Timer0 overflows at: F_CPU / (prescaler * 256)
    // For F_CPU=16MHz, prescaler=64 => overflow every 1.024 ms; we correct by counting
    // Simpler: increment g_millis by 1 every overflow for approximate millis (works ok for UI)
    g_millis++;
}

void timer0_init_millis(void)
{
    // set prescaler clk/64, normal mode
    TCCR0A = 0;
    TCCR0B = (1 << CS01) | (1 << CS00); // prescaler 64
    TIMSK0 = (1 << TOIE0);              // enable overflow interrupt
    TCNT0 = 0;
}

// use custom millis (not Arduino's)
uint32_t millis_custom(void)
{
    uint32_t m;
    cli();
    m = g_millis;
    sei();
    return m;
}

// -------------------- Timer1 for microsecond tick used by ISR --------------------
void timer1_init_for_micro(void)
{
    // prescaler = 8 => tick = 0.5 us; TCNT1 wraps at 65536
    TCCR1A = 0;
    TCCR1B = (1 << CS11); // clk/8
    TCNT1 = 0;
}

// -------------------- Timer2 carrier (38 kHz) setup --------------------
void timer2_init_carrier(void)
{
    // We'll use CTC toggling on OC2A: WGM21=1, COM2A0=0 initially (disabled)
    TCCR2A = (1 << WGM21);
    TCCR2B = (1 << CS20); // prescaler=1
    OCR2A = 210;          // approx -> 38kHz toggle (empirical)
    // ensure OC2A pin (PD3) is output
    DDRD |= (1 << PD3);
    // initially disable output compare toggle
    TCCR2A &= ~(1 << COM2A0);
    PORTD &= ~(1 << PD3);
}

// enable carrier (toggle OC2A)
static inline void carrier_enable(void)
{
    TCCR2A |= (1 << COM2A0);
}

// disable carrier
static inline void carrier_disable(void)
{
    TCCR2A &= ~(1 << COM2A0);
    PORTD &= ~(1 << PD3);
}

// -------------------- IR primitives (blocking) --------------------
void nec_mark_us(uint16_t us)
{
    carrier_enable();
    // use small loops to avoid long _delay_us inaccuracies
    while (us >= 100)
    {
        _delay_us(100);
        us -= 100;
    }
    while (us--)
        _delay_us(1);
}

void nec_space_us(uint16_t us)
{
    carrier_disable();
    while (us >= 100)
    {
        _delay_us(100);
        us -= 100;
    }
    while (us--)
        _delay_us(1);
}

// send 32-bit LSB-first NEC frame (blocking)
void nec_send32(uint32_t data)
{
    // leader
    nec_mark_us(NEC_LEADER_MARK);
    nec_space_us(NEC_LEADER_SPACE);

    for (uint8_t i = 0; i < 32; i++)
    {
        if (data & (1UL << i))
        {
            nec_mark_us(NEC_BIT_MARK);
            nec_space_us(NEC_ONE_SPACE);
        }
        else
        {
            nec_mark_us(NEC_BIT_MARK);
            nec_space_us(NEC_ZERO_SPACE);
        }
    }
    // final mark
    nec_mark_us(NEC_END_MARK);
    carrier_disable();
}

// pack and send two frames: [addr][~addr][cmd][x] and then [addr][~addr][cmd|0x80][y]
void send_payload_cmd(uint8_t cmd, uint8_t x, uint8_t y)
{
    uint8_t addr = 0x00;
    uint8_t addr_inv = ~addr;
    uint32_t frame1 = ((uint32_t)addr) | ((uint32_t)addr_inv << 8) | ((uint32_t)cmd << 16) | ((uint32_t)x << 24);
    nec_send32(frame1);
    _delay_ms(30); // small gap so receiver can re-sync
    uint8_t cmd2 = cmd | CMD_FRAME_SECOND_FLAG;
    uint32_t frame2 = ((uint32_t)addr) | ((uint32_t)addr_inv << 8) | ((uint32_t)cmd2 << 16) | ((uint32_t)y << 24);
    nec_send32(frame2);
}

// -------------------- IR receive ISR: measure edge durations on INT0 (D2) --------------------
// INT0 configured as any logical change; we measure TCNT1 and push durations (in microseconds)
// TCNT1 ticks at prescaler 8 -> 0.5us per tick -> convert later
ISR(INT0_vect)
{
    static uint16_t lastTimer = 0;
    uint16_t now = TCNT1;
    uint16_t delta = now - lastTimer; // unsigned wrap handles overflow
    lastTimer = now;

    // convert ticks (0.5us) to microseconds: durUs = delta * 0.5
    // store as integer microseconds (approx)
    uint32_t durUs = (uint32_t)delta / 2; // integer division gives 0.5us-> truncated, ok
    // better: approximate by (delta * 500) / 1000 but integer ok
    uint8_t next = (pulseHead + 1) % PULSE_BUF_SIZE;
    if (next != pulseTail)
    {
        pulseBuf[pulseHead] = durUs;
        pulseHead = next;
    }
}

// -------------------- try to decode a 32-bit NEC frame from pulse buffer (called in main) ----
bool tryDecodeNECFrame(uint32_t *outFrame)
{
    // copy indices atomically
    cli();
    uint8_t localTail = pulseTail;
    uint8_t localHead = pulseHead;
    sei();

    if (localTail == localHead)
        return false;

    // search for leader mark+space pattern
    uint8_t i = localTail;
    bool foundLeader = false;
    while (i != localHead)
    {
        uint32_t d1 = pulseBuf[i];
        uint8_t i2 = (i + 1) % PULSE_BUF_SIZE;
        if (i2 == localHead)
            break;
        uint32_t d2 = pulseBuf[i2];
        if (d1 > 7000 && d1 < 11000 && d2 > 3500 && d2 < 5500)
        {
            // found leader; start parsing after i2
            i = (i2 + 1) % PULSE_BUF_SIZE;
            foundLeader = true;
            break;
        }
        i = (i + 1) % PULSE_BUF_SIZE;
    }
    if (!foundLeader)
        return false;

    uint32_t value = 0;
    for (uint8_t b = 0; b < 32; b++)
    {
        if (i == localHead)
            return false;
        uint32_t mark = pulseBuf[i];
        i = (i + 1) % PULSE_BUF_SIZE;
        if (i == localHead)
            return false;
        uint32_t space = pulseBuf[i];
        i = (i + 1) % PULSE_BUF_SIZE;

        // validate mark (~560us)
        if (mark < 300 || mark > 900)
            return false;

        // decide bit by space length
        if (space > 1000)
            value |= (1UL << b);
    }

    // consume pulses up to i
    cli();
    pulseTail = i;
    sei();

    *outFrame = value;
    return true;
}

// -------------------- process decoded frame (two-frame scheme) --------------------
void processFrame(uint32_t frame)
{
    uint8_t b0 = frame & 0xFF;
    uint8_t b1 = (frame >> 8) & 0xFF;
    uint8_t b2 = (frame >> 16) & 0xFF; // cmd or cmd|0x80
    uint8_t b3 = (frame >> 24) & 0xFF; // payload (x or y)

    uint8_t cmd = b2;
    uint8_t payload = b3;

    if ((cmd & CMD_FRAME_SECOND_FLAG) == 0)
    {
        // first frame: store cmd and x
        pending_cmd = cmd;
        pending_x = payload;
        have_first_frame = true;
    }
    else
    {
        uint8_t cmd_naked = cmd & ~CMD_FRAME_SECOND_FLAG;
        if (have_first_frame && (cmd_naked == pending_cmd))
        {
            uint8_t x = pending_x;
            uint8_t y = payload;

            // dispatch only to the other role
            if (pending_cmd == CMD_BLUE_POS)
            {
                // blue pos was sent; the other unit should update blue ball
                if (ROLE == 2)
                {
                    blue_x = x;
                    blue_y = y;
                }
            }
            else if (pending_cmd == CMD_RED_POS)
            {
                if (ROLE == 1)
                {
                    red_x = x;
                    red_y = y;
                }
            }
            // reset
            have_first_frame = false;
        }
        else
        {
            have_first_frame = false;
        }
    }
}

// -------------------- Drawing --------------------
void drawScene(void)
{
    static int16_t last_bx = -1, last_by = -1, last_rx = -1, last_ry = -1;

    // erase previous (draw black over old circles)
    if (last_bx >= 0)
        tft.fillCircle(last_bx, last_by, BALL_RADIUS + 1, ILI9341_BLACK);
    if (last_rx >= 0)
        tft.fillCircle(last_rx, last_ry, BALL_RADIUS + 1, ILI9341_BLACK);

    // draw new
    tft.fillCircle(blue_x, blue_y, BALL_RADIUS, ILI9341_BLUE);
    tft.fillCircle(red_x, red_y, BALL_RADIUS, ILI9341_RED);

    last_bx = blue_x;
    last_by = blue_y;
    last_rx = red_x;
    last_ry = red_y;
}

// -------------------- Initialization helpers --------------------
void spi_setup_and_tft_init(void)
{
    // SPI.begin uses Arduino SPI implementation (allowed)
    SPI.begin();
    tft.begin();
    tft.setRotation(1);
    tft.fillScreen(ILI9341_BLACK);
}

// -------------------- Setup and main loop --------------------
int main(void)
{
    // initialize UART
    uart_init(9600);

    // timers
    timer0_init_millis();
    timer1_init_for_micro();
    timer2_init_carrier();

    // initialize display (uses SPI lib)
    spi_setup_and_tft_init();

    // init nunchuk (library function)
    bool n = nunchuk_begin(0x52); // typical address
    if (!n)
    {
        uart_send_string("Nunchuk init failed\n");
    }
    else
    {
        uart_send_string("Nunchuk ready\n");
    }

    // enable global interrupts after timer setup
    sei();

    // attach INT0 (D2) on any logical change (we set registers directly)
    EICRA |= (1 << ISC00); // ISC01=0, ISC00=1 -> any logical change
    EIMSK |= (1 << INT0);

    // initial draw
    drawScene();

    // main loop
    while (1)
    {
        // 1) Read nunchuk state
        if (nunchuk_get_state(0x52))
        {
            // nunchuk state struct in nunchuk.h: state.joy_x_axis / state.joy_y_axis
            uint8_t nx = state.joy_x_axis;
            uint8_t ny = state.joy_y_axis;

            // map to screen coords
            uint8_t mapped_x = (uint8_t)map_int32(nx, 0, 255, BALL_RADIUS, SCREEN_W - 1 - BALL_RADIUS);
            uint8_t mapped_y = (uint8_t)map_int32(ny, 0, 255, BALL_RADIUS, SCREEN_H - 1 - BALL_RADIUS);

            uint32_t now = millis_custom();
            if (ROLE == 1)
            {
                // controller for BLUE
                if (mapped_x != blue_x || mapped_y != blue_y)
                {
                    blue_x = mapped_x;
                    blue_y = mapped_y;
                    if ((now - lastSendMs) > SEND_INTERVAL_MS)
                    {
                        send_payload_cmd(CMD_BLUE_POS, blue_x, blue_y);
                        lastSendMs = now;
                        uart_send_string("Sent BLUE pos\n");
                    }
                }
            }
            else
            {
                // controller for RED
                if (mapped_x != red_x || mapped_y != red_y)
                {
                    red_x = mapped_x;
                    red_y = mapped_y;
                    if ((now - lastSendMs) > SEND_INTERVAL_MS)
                    {
                        send_payload_cmd(CMD_RED_POS, red_x, red_y);
                        lastSendMs = now;
                        uart_send_string("Sent RED pos\n");
                    }
                }
            }
        }

        // 2) decode any incoming NEC frames and process
        uint32_t frame;
        while (tryDecodeNECFrame(&frame))
        {
            processFrame(frame);
        }

        // 3) update display
        drawScene();

        // small sleep
        _delay_ms(10);
    }

    return 0;
}
