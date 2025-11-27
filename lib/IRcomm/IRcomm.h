#ifndef IRCOMM_H
#define IRCOMM_H

#include <stdint.h>

// Buffer size limit for messages
#define IR_MAX_MSG_LEN 32

// Initialize the IR system (Timers, Pins, ISRs)
void ir_init();

// Send a text string (Non-blocking)
void ir_send(char *str);

// Check if a new message has been received
// Returns 1 if yes, 0 if no
uint8_t ir_available();

// Copy the received message into your buffer
// Returns length of message
uint8_t ir_read(char *buffer);

// Internal update function (Must be called in main loop if not using ISR-only logic)
void ir_update();

#endif