#ifndef TFT_H
#define TFT_H

#include <stdint.h>

// prototypes
void tft_init(void);
void tft_fill_screen(uint16_t color);
void tft_draw_pixel(uint16_t x, uint16_t y, uint16_t color);
void tft_fill_circle(uint16_t x0, uint16_t y0, uint8_t r, uint16_t color);

// RGB565 helper (inline, beschikbaar in alle files)
static inline uint16_t rgb565(uint8_t r, uint8_t g, uint8_t b)
{
    return ((r & 0xF8) << 8) |
           ((g & 0xFC) << 3) |
           ((b & 0xF8) >> 3);
}

#endif
