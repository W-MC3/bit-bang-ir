#ifndef IRCOMM_H
#define IRCOMM_H

#include <stdint.h>

#ifdef __cplusplus
extern "C"
{
#endif

#define IR_MAX_MSG_LEN 32

    // Setup
    void ir_init();

    // Zenden & Ontvangen
    void ir_send(const char *str);
    uint8_t ir_available();
    uint8_t ir_read(char *buffer);

    // Core functies
    void ir_update();

    // NIEUW: Onze eigen tijd functie
    unsigned long ir_millis();

#ifdef __cplusplus
}
#endif

#endif