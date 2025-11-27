#include <Wire.h>
#include <Adafruit_GFX.h>
#include <Adafruit_ILI9341.h>

// Include the IR Library
extern "C"
{
#include "IRComm.h"
}

#define WHITE 0xFFFF
#define RED 0xF800
#define BLUE 0x001F

#define TFT_CS 10
#define TFT_DC 9
#define TFT_RST 8

Adafruit_ILI9341 tft(TFT_CS, TFT_DC, TFT_RST);

// --- FORWARD DECLARATION (This fixes the error) ---
void drawBall(int x, int y, uint16_t color);

uint8_t joyX_raw = 128;
uint8_t joyY_raw = 128;

int myX = 160, myY = 120;
int prevMyX = 160, prevMyY = 120;

int remoteX = 160, remoteY = 120;
int prevRemoteX = 160, prevRemoteY = 120;

unsigned long lastSend = 0;

// Buffers
char msgOut[32];
char msgIn[32];

void nunchuckInit()
{
    Wire.beginTransmission(0x52);
    Wire.write(0xF0);
    Wire.write(0x55);
    Wire.endTransmission();
    delay(1);
    Wire.beginTransmission(0x52);
    Wire.write(0xFB);
    Wire.write(0x00);
    Wire.endTransmission();
    delay(1);
}

void readNunchuck()
{
    Wire.beginTransmission(0x52);
    Wire.write(0x00);
    Wire.endTransmission();
    delayMicroseconds(1000);
    Wire.requestFrom(0x52, 6);
    if (Wire.available() == 6)
    {
        joyX_raw = Wire.read();
        joyY_raw = Wire.read();
        Wire.read();
        Wire.read();
        Wire.read();
        Wire.read();
    }
}

void setup()
{
    ir_init();

    Wire.begin();
    nunchuckInit();
    tft.begin();
    tft.setRotation(1);
    tft.fillScreen(WHITE);

    // Now this works because we declared it above
    drawBall(myX, myY, RED);
    drawBall(remoteX, remoteY, BLUE);
}

void loop()
{
    // 1. Update IR
    ir_update();

    // 2. Read Input
    readNunchuck();
    int joyX = map(joyX_raw, 0, 255, 160 + 100, 160 - 100);
    int joyY = map(joyY_raw, 0, 255, 120 + 100, 120 - 100);

    prevMyX = myX;
    prevMyY = myY;
    myX = constrain(joyX, 0, tft.width() - 1);
    myY = constrain(joyY, 0, tft.height() - 1);

    // 3. Send
    if (millis() - lastSend >= 50)
    {
        sprintf(msgOut, "%d,%d", myX, myY);
        ir_send(msgOut);
        lastSend = millis();
    }

    // 4. Receive
    if (ir_available())
    {
        ir_read(msgIn);
        char *commaPtr = strchr(msgIn, ',');

        if (commaPtr != NULL)
        {
            *commaPtr = 0;
            int rxX = atoi(msgIn);
            int rxY = atoi(commaPtr + 1);

            prevRemoteX = remoteX;
            prevRemoteY = remoteY;
            remoteX = constrain(rxX, 0, tft.width() - 1);
            remoteY = constrain(rxY, 0, tft.height() - 1);
        }
    }

    // 5. Draw
    if (myX != prevMyX || myY != prevMyY)
    {
        drawBall(prevMyX, prevMyY, WHITE);
        drawBall(myX, myY, RED);
    }
    if (remoteX != prevRemoteX || remoteY != prevRemoteY)
    {
        drawBall(prevRemoteX, prevRemoteY, WHITE);
        drawBall(remoteX, remoteY, BLUE);
    }
}

void drawBall(int x, int y, uint16_t color)
{
    tft.fillCircle(x, y, 6, color);
}