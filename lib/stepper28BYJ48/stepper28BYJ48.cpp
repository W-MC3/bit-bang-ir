/****************************************************************************************
 * File:        stepper28BYJ48.cpp
 * Description: Driver for the 28BYJ-48 unipolar stepper motor with ULN2003 driver.
 *              Uses half-step mode (4096 steps/revolution).
 *              Compatible with ESP32 and other Arduino-framework platforms.
 ****************************************************************************************/

#include "stepper28BYJ48.h"
#include <Arduino.h>

/* Half-step sequence for the four coils (IN1..IN4).
 * Each row is one step; columns map to pin1..pin4. */
static const uint8_t HALF_STEP_SEQ[8][4] = {
    {1, 0, 0, 0},
    {1, 1, 0, 0},
    {0, 1, 0, 0},
    {0, 1, 1, 0},
    {0, 0, 1, 0},
    {0, 0, 1, 1},
    {0, 0, 0, 1},
    {1, 0, 0, 1},
};

static uint8_t s_pins[4];
static int8_t  s_step_index = 0; /* current position in the 8-step sequence */

void stepper_init(uint8_t pin1, uint8_t pin2, uint8_t pin3, uint8_t pin4)
{
    s_pins[0] = pin1;
    s_pins[1] = pin2;
    s_pins[2] = pin3;
    s_pins[3] = pin4;

    for (uint8_t i = 0; i < 4; i++)
    {
        pinMode(s_pins[i], OUTPUT);
        digitalWrite(s_pins[i], LOW);
    }

    s_step_index = 0;
}

void stepper_step(int32_t steps, uint8_t speed_rpm)
{
    if (speed_rpm == 0)
        return;

    /* Delay between steps in microseconds:
     *   delay_us = 60,000,000 / (speed_rpm * STEPPER_STEPS_PER_REV) */
    uint32_t delay_us = 60000000UL / ((uint32_t)speed_rpm * STEPPER_STEPS_PER_REV);

    int8_t direction = (steps >= 0) ? 1 : -1;
    int32_t abs_steps = (steps >= 0) ? steps : -steps;

    for (int32_t i = 0; i < abs_steps; i++)
    {
        s_step_index = (s_step_index + direction + 8) % 8;

        for (uint8_t j = 0; j < 4; j++)
        {
            digitalWrite(s_pins[j], HALF_STEP_SEQ[s_step_index][j]);
        }

        /* Use delay() for large intervals to avoid delayMicroseconds() overflow
         * on platforms where it wraps around at ~16 383 µs. */
        if (delay_us > 16383)
        {
            delay(delay_us / 1000);
        }
        else
        {
            delayMicroseconds(delay_us);
        }
    }
}

void stepper_stop(void)
{
    for (uint8_t i = 0; i < 4; i++)
    {
        digitalWrite(s_pins[i], LOW);
    }
}
