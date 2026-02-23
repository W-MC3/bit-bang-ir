/****************************************************************************************
 * File:        stepper28BYJ48.h
 * Description: Driver for the 28BYJ-48 unipolar stepper motor with ULN2003 driver.
 *              Uses half-step mode (4096 steps/revolution).
 *              Compatible with ESP32 and other Arduino-framework platforms.
 ****************************************************************************************/

#ifndef STEPPER28BYJ48_H
#define STEPPER28BYJ48_H

#include <stdint.h>

#ifdef __cplusplus
extern "C"
{
#endif

/** Total half-steps for one full output shaft revolution.
 *  Step angle = 5.625° / 64 ≈ 0.0879°  →  360 / 0.0879 ≈ 4096 steps/rev. */
#define STEPPER_STEPS_PER_REV 4096

/**
 * @brief Initialise the stepper motor by configuring the four output pins.
 *
 * @param pin1  Arduino pin number connected to ULN2003 IN1.
 * @param pin2  Arduino pin number connected to ULN2003 IN2.
 * @param pin3  Arduino pin number connected to ULN2003 IN3.
 * @param pin4  Arduino pin number connected to ULN2003 IN4.
 */
void stepper_init(uint8_t pin1, uint8_t pin2, uint8_t pin3, uint8_t pin4);

/**
 * @brief Move the motor a given number of steps at the requested speed.
 *
 * @param steps     Number of steps to move. Positive = clockwise,
 *                  negative = counter-clockwise.
 * @param speed_rpm Rotation speed in revolutions per minute (1–15 RPM recommended
 *                  for the 28BYJ-48).
 */
void stepper_step(int32_t steps, uint8_t speed_rpm);

/**
 * @brief Power off all coils to reduce heat and current draw when idle.
 */
void stepper_stop(void);

#ifdef __cplusplus
}
#endif

#endif /* STEPPER28BYJ48_H */
