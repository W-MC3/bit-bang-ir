/****************************************************************************************
* File:         Nunchuk.h
* Authors:       bRAM, Michiel Dirks
* Created on:   20-11-2025
* Company:      Windesheim
* Website:      https://www.windesheim.nl/opleidingen/voltijd/bachelor/ict-zwolle
****************************************************************************************/

#ifndef NUNCHUK_H_
#define NUNCHUK_H_

#include <inttypes.h>
#include <stdbool.h>

// don't encode
#define ENCODED         0
#define IDLEN		4 // bytes

typedef struct {
	uint8_t		joy_x_axis;
	uint8_t		joy_y_axis;
	uint16_t	accel_x_axis;
	uint16_t	accel_y_axis;
	uint16_t	accel_z_axis;
	uint8_t		z_button;
	uint8_t		c_button;
} s_ncState;

typedef struct {
	uint16_t	x0;
	uint16_t	y0;
	uint16_t	z0;
	uint16_t	x1;
	uint16_t	y1;
	uint16_t	z1;
	uint8_t		xmin;
	uint8_t		xmax;
	uint8_t		xcenter;
	uint8_t		ymin;
	uint8_t		ymax;
	uint8_t		ycenter;
	uint16_t	chksum;
} s_ncCal;

char id[2*IDLEN+3]; // '0xAABBCCDD\0'
s_ncState state;
s_ncCal cal;

bool nunchuk_begin(uint8_t address);

bool nunchuk_get_state(uint8_t address);

bool nunchuk_get_calibration(uint8_t address);

#endif
