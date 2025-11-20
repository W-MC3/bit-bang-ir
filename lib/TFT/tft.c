#include "tft.h"
#include <avr/io.h>
#include <util/delay.h>

// === Pins (ATmega328P) ===
#define TFT_CS PB2
#define TFT_DC PB1
#define TFT_RST PB0
#define MOSI PB3
#define SCK PB5

// === Colors (RGB565) ===
#define COLOR_WHITE 0xFFFF
#define COLOR_BLUE 0x001F

// === SPI helpers ===
static inline void spi_init(void)
{
    DDRB |= (1 << MOSI) | (1 << SCK);
    SPCR = (1 << SPE) | (1 << MSTR) | (1 << SPR0); // Master, Fclk/16
}

static inline void spi_write(uint8_t data)
{
    SPDR = data;
    while (!(SPSR & (1 << SPIF)))
        ;
}

// === TFT helpers ===
static inline void tft_select(void) { PORTB &= ~(1 << TFT_CS); }
static inline void tft_deselect(void) { PORTB |= (1 << TFT_CS); }
static inline void tft_command(uint8_t cmd)
{
    PORTB &= ~(1 << TFT_DC);
    tft_select();
    spi_write(cmd);
    tft_deselect();
}
static inline void tft_data(uint8_t data)
{
    PORTB |= (1 << TFT_DC);
    tft_select();
    spi_write(data);
    tft_deselect();
}

// === TFT low-level init sequence (ILI9341) ===
void tft_init(void)
{
    DDRB |= (1 << TFT_CS) | (1 << TFT_DC) | (1 << TFT_RST);
    PORTB |= (1 << TFT_RST);
    _delay_ms(100);
    PORTB &= ~(1 << TFT_RST);
    _delay_ms(100);
    PORTB |= (1 << TFT_RST);
    _delay_ms(100);

    spi_init();

    // Basic ILI9341 init (minimal)
    tft_command(0x01); // Software reset
    _delay_ms(50);
    tft_command(0x11); // Sleep out
    _delay_ms(120);
    tft_command(0x29); // Display ON
    _delay_ms(50);
}

// Set address window
static void tft_set_window(uint16_t x0, uint16_t y0, uint16_t x1, uint16_t y1)
{
    tft_command(0x2A); // Column addr
    tft_data(x0 >> 8);
    tft_data(x0 & 0xFF);
    tft_data(x1 >> 8);
    tft_data(x1 & 0xFF);

    tft_command(0x2B); // Page addr
    tft_data(y0 >> 8);
    tft_data(y0 & 0xFF);
    tft_data(y1 >> 8);
    tft_data(y1 & 0xFF);

    tft_command(0x2C); // Memory write
}

// === Draw pixel ===
void tft_draw_pixel(uint16_t x, uint16_t y, uint16_t color)
{
    tft_set_window(x, y, x, y);
    tft_data(color >> 8);
    tft_data(color & 0xFF);
}

// === Fill screen ===
void tft_fill_screen(uint16_t color)
{
    tft_set_window(0, 0, 239, 319); // 240x320
    for (uint32_t i = 0; i < 240UL * 320UL; i++)
    {
        tft_data(color >> 8);
        tft_data(color & 0xFF);
    }
}

// === Draw filled circle (Midpoint circle) ===
void tft_fill_circle(uint16_t x0, uint16_t y0, uint8_t r, uint16_t color)
{
    int16_t f = 1 - r;
    int16_t ddF_x = 1;
    int16_t ddF_y = -2 * r;
    int16_t x = 0;
    int16_t y = r;

    // draw initial center line
    for (int16_t i = y0 - r; i <= y0 + r; i++)
        tft_draw_pixel(x0, i, color);

    while (x < y)
    {
        if (f >= 0)
        {
            y--;
            ddF_y += 2;
            f += ddF_y;
        }
        x++;
        ddF_x += 2;
        f += ddF_x;

        for (int16_t i = y0 - y; i <= y0 + y; i++)
            tft_draw_pixel(x0 + x, i, color);
        for (int16_t i = y0 - y; i <= y0 + y; i++)
            tft_draw_pixel(x0 - x, i, color);
        for (int16_t i = y0 - x; i <= y0 + x; i++)
            tft_draw_pixel(x0 + y, i, color);
        for (int16_t i = y0 - x; i <= y0 + x; i++)
            tft_draw_pixel(x0 - y, i, color);
    }
}
