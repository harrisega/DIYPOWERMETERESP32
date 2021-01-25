/*
Copyright (c) 2020 Harris Ega

Permission is hereby granted, free of charge, to any person obtaining a copy of
this software and associated documentation files (the “Software”), to deal in
the Software without restriction, including without limitation the rights to
use, copy, modify, merge, publish, distribute, sublicense, and/or sell copies of
the Software, and to permit persons to whom the Software is furnished to do so,
subject to the following conditions:

The above copyright notice and this permission notice shall be included in all
copies or substantial portions of the Software.

THE SOFTWARE IS PROVIDED “AS IS”, WITHOUT WARRANTY OF ANY KIND, EXPRESS OR
IMPLIED, INCLUDING BUT NOT LIMITED TO THE WARRANTIES OF MERCHANTABILITY, FITNESS
FOR A PARTICULAR PURPOSE AND NONINFRINGEMENT. IN NO EVENT SHALL THE AUTHORS OR
COPYRIGHT HOLDERS BE LIABLE FOR ANY CLAIM, DAMAGES OR OTHER LIABILITY, WHETHER
IN AN ACTION OF CONTRACT, TORT OR OTHERWISE, ARISING FROM, OUT OF OR IN
CONNECTION WITH THE SOFTWARE OR THE USE OR OTHER DEALINGS IN THE SOFTWARE.
*/

/*
 * DIYPowerMeterESP32.h
 *
 * Library for DIY Power Meter ESP32
 * Based on the PZEM004T library by @olehs https://github.com/olehs/PZEM004T and Jakub Mandula https://github.com/mandulaj
 * Author: Harris Ega
 *
 *
*/

#ifndef DIYPOWERMETERESP32_H
#define DIYPOWERMETERESP32_H

#if defined(ARDUINO) && ARDUINO >= 100
#include "Arduino.h"
#else
#include "WProgram.h"
#endif

/* Pin Mapping */
const int ledACT = 0;          // IO0, Output, Green LED, Active Low
const int ledNET = 5;          // IO5, Output, Blue LED, Active Low
const int ledRLY = 15;         // IO15, Output, Red LED, Active Low
const int relayLatchReset = 2; // IO2, Output, Active Low
const int relayLatchSet = 12;  // IO12, Output, Active Low
const int buzzerPin = 18;      // IO18, Output, Active Low
const int button1Pin = 35;     // IN35, Input, Active Low
const int button2Pin = 34;     // IN34, Input, Active Low

#define POWERMETER_SERIAL_INTERFACE &Serial2
#define POWERMETER_DEFAULT_ADDRESS 0x01
#define POWERMETER_DEFAULT_FRAME_FORMAT POWERMETER_COMM_8N1
#define POWERMETER_DEFAULT_BAUDRATE POWERMETER_BAUD_19200

#define READ_TIMEOUT 500
#define WRITE_TIMEOUT 5000

/* Power Meter Communication Parameters */
// High Nibble
enum enumPowerMeterConfigFrameFormat
{
    POWERMETER_COMM_8N1 = 0x00,
    POWERMETER_COMM_8E1 = 0x40,
    POWERMETER_COMM_8O1 = 0x80,
    POWERMETER_COMM_8N2 = 0xF0
};

// Low Nibble
enum enumPowerMeterConfigBaudrate
{
    POWERMETER_BAUD_1200 = 0x03,
    POWERMETER_BAUD_2400 = 0x04,
    POWERMETER_BAUD_4800 = 0x05,
    POWERMETER_BAUD_9600 = 0x06,
    POWERMETER_BAUD_19200 = 0x07
};

enum enumLedSelect
{
    LED_ACT = 1,
    LED_NET = 2,
    LED_RLY = 3
};

enum enumState
{
    OFF = 0,
    ON = 1
};

enum enumButtonState
{
    NOT_PRESSED = 0,
    PRESSED = 1
};

enum enumButtonID
{
    BUTTON1 = 0,
    BUTTON2 = 1
};

class DIYPOWERMETERESP32
{
public:
    HardwareSerial *_serialPort; // Serial Port Interface

    DIYPOWERMETERESP32(HardwareSerial *port, uint8_t addr = POWERMETER_DEFAULT_ADDRESS);
    ~DIYPOWERMETERESP32();

    /* Power Meter Section */
    bool readMeasurement();

    float voltage();
    float current();
    float power();
    float energy();
    float frequency();
    float pf();
    float co2();
    float coreTemperature();

    uint8_t getAddress();
    uint8_t getCommSettings();

    bool setConfig(uint8_t slaveAddr = 0x01, enumPowerMeterConfigFrameFormat frameFormat = POWERMETER_DEFAULT_FRAME_FORMAT, enumPowerMeterConfigBaudrate baudrate = POWERMETER_DEFAULT_BAUDRATE);
    bool getConfig();
    void checkConfig();

    bool resetEnergy();

    /* Control IO Section */
    void setLED(enumLedSelect ledTarget, enumState ledState);
    void setRelay(enumState relayState);
    void setBuzzer(enumState buzzerState);
    enumButtonState getButton(enumButtonID buttonID);

private:
    Stream *_serial;       // Serial Stream Interface
    uint8_t _addr;         // Device address
    uint8_t _commSettings; // Communication Settings (UART Frame Format and BaudRate)

    struct
    {
        float voltage;
        float current;
        float power;
        float energy;
        float frequency;
        float pf;
        float co2;
        float coreTemperature;
    } _currentValues; // Measured Values

    uint64_t _lastRead; // Last time values were updated

    void updateInterfaceBaudrate(unsigned long baud);

    void init(uint8_t addr);                                                        // Init
    bool updateValues();                                                            // Get latest values from device registers
    uint16_t receive(uint8_t *resp, uint16_t len, uint32_t timeout = READ_TIMEOUT); // Receive len bytes to buffer

    bool sendCmd(uint8_t cmd, uint16_t regAddr, uint16_t *value, uint16_t dataLen = 0, bool check = false, uint16_t slave_addr = 0xFFFF);

    void setCRC(uint8_t *buf, uint16_t len);
    bool checkCRC(const uint8_t *buf, uint16_t len);

    uint16_t CRC16(const uint8_t *data, uint16_t len); // Calculate CRC of Buffer
};

#endif