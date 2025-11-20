/****************************************************************************************
* File:         Nunchuk.c
* Authors:       bRAM, Michiel Dirks
* Created on:   20-11-2025
* Company:      Windesheim
* Website:      https://www.windesheim.nl/opleidingen/voltijd/bachelor/ict-zwolle
****************************************************************************************/

/*
 * nunchuk library
 * http://wiibrew.org/wiki/Wiimote/Extension_Controllers/Nunchuk
 * http://create.arduino.cc/projecthub/infusion/using-a-wii-nunchuk-with-arduino-597254
 * https://www.robotshop.com/media/files/PDF/inex-zx-nunchuk-datasheet.pdf
 *
 * Nunchuk has 256 byte memory, which can be requested by setting
 * an offset (write) and requesting a defined amount of data (read)
 * The amount of data appears to be max 32 bytes
 * 
 * bRAM, Michiel
 */
#include <util/delay.h>
#include "../../src/HAL/I2C/twi.h"
#include "nunchuk.h"

// nunchuk memory addresses
#define NCSTATE	0x00	// address of state (6 bytes)
#define NCCAL	0x20	// address of callibration data (16 bytes)
#define NCID	0xFA	// address of id (4 bytes)

#define CHUNKLEN	32
#define STATELEN	6
#define CALLEN		16

#define WAITFORREAD	1	// ms

// nibble to hex ascii
char btoa[] = {'0', '1', '2', '3', '4', '5', '6', '7',
		'8', '9', 'A', 'B', 'C', 'D', 'E', 'F'};

/* ---- initialize variables ---- */
uint8_t buffer[CHUNKLEN];

/* ---- forward declarations ----*/
static bool nunchuk_get_id(uint8_t address);
static uint8_t nunchuk_read(uint8_t address, uint8_t offset, uint8_t len);

/* ---- public functions ---- */
 
/*
 * do the handschake
 */
bool nunchuk_begin(uint8_t address) {

	TWI_MT_Start();
	TWI_Transmit_SLAW(address);

	if (ENCODED) {
		TWI_Transmit_Byte(0x40);
		TWI_Transmit_Byte(0x00);
	}
	else {
		TWI_Transmit_Byte(0xF0);
		TWI_Transmit_Byte(0x55);
		TWI_Stop();

		_delay_ms(1);

		TWI_MT_Start();
		TWI_Transmit_SLAW(address);

		TWI_Transmit_Byte(0xFB);
		TWI_Transmit_Byte(0x00);
	}

	TWI_Stop();

	// get the id
	if (!nunchuk_get_id(address))
		return false;

	return true;
}

/*
 * get current state
 * state encoding:
 *	byte 0: SX[7:0]
 *	byte 1: SY[7:0]
 *	byte 2: AX[9:2]
 *	byte 3: AY[9:2]
 *	byte 4: AZ[9:2]
 *	byte 5: AZ[1:0], AY[1:0], AX[1:0], BC, BZ
 */
bool nunchuk_get_state(uint8_t address) {
	// read state from memory address
	if (nunchuk_read(address, NCSTATE, STATELEN) != STATELEN)
		return false;

	// set parameters
	state.joy_x_axis = buffer[0];
	state.joy_y_axis = buffer[1];
	state.accel_x_axis = (buffer[2] << 2) | ((buffer[5] & 0x0C) >> 2);
	state.accel_y_axis = (buffer[3] << 2) | ((buffer[5] & 0x30) >> 4);
	state.accel_z_axis = (buffer[4] << 2) | ((buffer[5] & 0xC0) >> 6);
	/* 0 = pressed */
	state.z_button = !(buffer[5] & 0x01);
	state.c_button = !((buffer[5] & 0x02) >> 1);

	return true;
}

/*
 * get calibration data
 * calibration encoding 0G eand 1G
 *	byte 0: X0[9:2]
 *	byte 1: Y0[9:2]
 *	byte 2: Z0[9:2]
 *	byte 3: LSB_XYZ0[1:0]
 *	byte 4: X1[9:2]
 *	byte 5: Y1[9:2]
 *	byte 6: Z1[9:2]
 *	byte 7: LSB_XYZ1[1:0]
 *	byte 8: XMIN[7:0]
 *	byte 9: XMAX[7:0]
 *	byte 10: XCENTER[7:0]
 *	byte 11: YMIN[7:0]
 *	byte 12: YMAX[7:0]
 *	byte 13: YCENTER[7:0]
 *	byte 14: CHKSUM1[7:0]
 *	byte 15: CHKSUM2[7:0]
 */
bool nunchuk_get_calibration(uint8_t address) {
	// read state from memory address
	if (nunchuk_read(address, NCCAL, CALLEN) != CALLEN)
		return false;

	// set parameters
	cal.x0 = (buffer[0] << 2) | ((buffer[3] & 0x03));
	cal.y0 = (buffer[1] << 2) | ((buffer[3] & 0x0B) >> 2);
	cal.z0 = (buffer[2] << 2) | ((buffer[3] & 0x30) >> 4);
	cal.x1 = (buffer[4] << 2) | ((buffer[7] & 0x03));
	cal.y1 = (buffer[5] << 2) | ((buffer[7] & 0x0B) >> 2);
	cal.z1 = (buffer[6] << 2) | ((buffer[7] & 0x30) >> 4);
	cal.xmin = buffer[8];
	cal.xmax = buffer[9];
	cal.xcenter = buffer[10];
	cal.ymin = buffer[11];
	cal.ymax = buffer[12];
	cal.ycenter = buffer[13];
	cal.chksum = (buffer[14]<<8)|buffer[15];

	return true;
}

/* ---- private functions ---- */

/*
 * get the device id (nunchuk should be 0xA4200000)
 */
static bool nunchuk_get_id(uint8_t address) {
	// read data from address
	if(nunchuk_read(address, NCID, IDLEN) != IDLEN)
		return false;

	// copy buffer to id string
	id[0] = '0';
	id[1] = 'x';
	for (uint8_t i=0; i < IDLEN; i++) {
		id[2+2*i] = btoa[(buffer[i]>>4)];
		id[2+2*i+1] = btoa[(buffer[i]&0x0F)];
	}
	id[2*IDLEN+2] = '\0';

	return true;
}

/*
 * decode byte
 */
static uint8_t nunchuk_decode(uint8_t b)
{
	return (b^0x17) + 0x17;
}

/*
 * read buffer
 */
static uint8_t nunchuk_read(uint8_t address, uint8_t offset, uint8_t len) {
	uint8_t n = 0;

	// send offset
	TWI_MT_Start();
	TWI_Transmit_SLAW(address);

    TWI_Transmit_Byte(offset);

	TWI_Stop();

	// wait untill arrived
	_delay_ms(WAITFORREAD);

	// request len bytes
	TWI_MT_Start();
	TWI_Transmit_SLAR(address);


	// read bytes
	for (n = 0; n < len; n++) {
		// Read n-th byte
		bool ack = n < (len - 1);
		buffer[n] = TWI_Receive_Byte(ack);
		if (ENCODED)
			buffer[n] = nunchuk_decode(buffer[n]);
	}

	TWI_Stop();

	/* return nr bytes */
	return n;
}
