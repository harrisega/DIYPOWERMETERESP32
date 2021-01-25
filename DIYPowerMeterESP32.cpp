#include "DIYPowerMeterESP32.h"
#include <pgmspace.h>

#define REG_VOLTAGE 0x0048     // 4 bytes, values in 0.0001V
#define REG_CURRENT 0x0049     // 4 bytes, values in 0.0001A
#define REG_POWER 0x004A       // 4 bytes, values in 0.0001W
#define REG_ENERGY 0x004B      // 4 bytes, values in 0.0001kWh
#define REG_PF 0x004C          // 4 bytes, values in 0.001
#define REG_CO2 0x004D         // 4 bytes, values in 0.0001kg
#define REG_TEMPERATURE 0x004E // 4 bytes, values in 0.01°C
#define REG_FREQUENCY 0x004F   // 4 bytes, values in 0.01Hz
#define REG_CONFIG 0x0004      // 2 bytes, Slave Address and Communication Settings (UART FORMAT and BAUD RATE)

#define CMD_RHR 0x03 // Read Holding Registers
#define CMD_RIR 0x04 // Read Input Registers
#define CMD_WSR 0x06 // Write Single Register
#define CMD_WMR 0x10 // Write Multiple Register

#define UPDATE_TIME 300

#define RELAY_LATCH_PULSE_DURATION_MS 100 // min. 50ms Pulse Duration

extern HardwareSerial Serial;

#define DEBUG
void printBuf(uint8_t *buffer, uint16_t len)
{
#ifdef DEBUG
    for (uint16_t i = 0; i < len; i++)
    {
        char temp[6];
        sprintf(temp, "%02X ", buffer[i]);
        Serial.print(temp);
    }
    Serial.println();
#endif
}

/*!
 * DIYPOWERMETERESP32::DIYPOWERMETERESP32
 *
 * Hardware serial constructor
 *
 * @param port Hardware serial to use
 * @param addr Slave address of device
*/
DIYPOWERMETERESP32::DIYPOWERMETERESP32(HardwareSerial *port, uint8_t addr)
{
    unsigned long baudrate;
    switch (POWERMETER_DEFAULT_BAUDRATE)
    {
    case POWERMETER_BAUD_1200:
        baudrate = 1200;
        break;
    case POWERMETER_BAUD_2400:
        baudrate = 2400;
        break;
    case POWERMETER_BAUD_4800:
        baudrate = 4800;
        break;
    case POWERMETER_BAUD_9600:
        baudrate = 9600;
        break;
    case POWERMETER_BAUD_19200:
        baudrate = 19200;
        break;
    }
    _serialPort = port;
    port->begin(baudrate);
    this->_serial = port;
    init(addr);
}

/*!
 * DIYPOWERMETERESP32::~DIYPOWERMETERESP32
 *
 * Destructor deleting software serial
 *
*/
DIYPOWERMETERESP32::~DIYPOWERMETERESP32()
{
}

/*!
 * DIYPOWERMETERESP32::updateInterfaceBaudrate
 *
 * Update current interface baudrate
 *
 * @return status
*/
void DIYPOWERMETERESP32::updateInterfaceBaudrate(unsigned long baud)
{
    _serialPort->updateBaudRate(baud);
}

/*!
 * DIYPOWERMETERESP32::readMeasurement
 *
 * Get current measurement readings
 *
 * @return status
*/
bool DIYPOWERMETERESP32::readMeasurement()
{
    return updateValues();
}

/*!
 * DIYPOWERMETERESP32::voltage
 *
 * Get line voltage in Volts
 *
 * @return current L-N volage
*/
float DIYPOWERMETERESP32::voltage()
{
    return _currentValues.voltage;
}

/*!
 * DIYPOWERMETERESP32::current
 *
 * Get line in Amps
 *
 * @return line current
*/
float DIYPOWERMETERESP32::current()
{
    return _currentValues.current;
}

/*!
 * DIYPOWERMETERESP32::power
 *
 * Get Active power in W
 *
 * @return active power in W
*/
float DIYPOWERMETERESP32::power()
{
    return _currentValues.power;
}

/*!
 * DIYPOWERMETERESP32::energy
 *
 * Get Active energy in kWh since last reset
 *
 * @return active energy in kWh
*/
float DIYPOWERMETERESP32::energy()
{
    return _currentValues.energy;
}

/*!
 * DIYPOWERMETERESP32::frequeny
 *
 * Get current line frequency in Hz
 *
 * @return line frequency in Hz
*/
float DIYPOWERMETERESP32::frequency()
{
    return _currentValues.frequency;
}

/*!
 * DIYPOWERMETERESP32::pf
 *
 * Get power factor of load
 *
 * @return load power factor
*/
float DIYPOWERMETERESP32::pf()
{
    return _currentValues.pf;
}

/*!
 * DIYPOWERMETERESP32::co2
 *
 * Get CO2 Estimation
 *
 * @return CO2 Estimation in kg
*/
float DIYPOWERMETERESP32::co2()
{
    return _currentValues.co2;
}

/*!
 * DIYPOWERMETERESP32::coreTemperature
 *
 * Get current power meter module core temperature
 *
 * @return current module core temperature in deg Celcius
*/
float DIYPOWERMETERESP32::coreTemperature()
{
    return _currentValues.coreTemperature;
}

/*!
 * DIYPOWERMETERESP32::setLED
 *
 * Set LED State
 *
 * @param led Led to be controlled, possible values : LED_ACT, LED_NET, LED_RLY
 * @param ledState Led state, possible values : OFF, ON
*/
void DIYPOWERMETERESP32::setLED(enumLedSelect led, enumState ledState)
{
    int ledIO = -1;
    switch (led)
    {
    case LED_ACT:
        ledIO = ledACT;
        break;
    case LED_NET:
        ledIO = ledNET;
        break;
    case LED_RLY:
        ledIO = ledRLY;
        break;
    }
    if (ledIO != -1)
    {
        digitalWrite(ledIO, ledState == ON ? LOW : HIGH);
    }
}

/*!
 * DIYPOWERMETERESP32::setRelay
 *
 * Set Relay State
 *
 * @param relayState Relay state, possible values : OFF, ON
*/
void DIYPOWERMETERESP32::setRelay(enumState relayState)
{
    int relayAction = -1;
    // Make sure initial condition both relayLatchSet and Reset in HIGH Condition
    digitalWrite(relayLatchReset, HIGH);
    digitalWrite(relayLatchSet, HIGH);

    relayAction = relayState == ON ? relayLatchSet : relayLatchReset;

    if (relayAction != -1)
    {
        digitalWrite(relayAction, LOW);
        delay(RELAY_LATCH_PULSE_DURATION_MS);
        digitalWrite(relayAction, HIGH);
        delay(RELAY_LATCH_PULSE_DURATION_MS);
    }
}

/*!
 * DIYPOWERMETERESP32::setBuzzer
 *
 * Set Buzzer State
 *
 * @param led Led to be controlled, possible values : LED_ACT, LED_NET, LED_RLY
 * @param ledState Led state, possible values : OFF, ON
*/
void DIYPOWERMETERESP32::setBuzzer(enumState buzzerState)
{
    digitalWrite(buzzerPin, buzzerState == ON ? LOW : HIGH);
}

/*!
 * DIYPOWERMETERESP32::getButton
 *
 * Set Buzzer State
 *
 * @param led Led to be controlled, possible values : LED_ACT, LED_NET, LED_RLY
 * @param ledState Led state, possible values : OFF, ON
*/
enumButtonState DIYPOWERMETERESP32::getButton(enumButtonID buttonID)
{
    int targetButtonID = -1;
    switch (buttonID)
    {
    case BUTTON1:
        targetButtonID = button1Pin;
        break;
    case BUTTON2:
        targetButtonID = button2Pin;
        break;
    }
    if (targetButtonID == -1)
        return NOT_PRESSED;

    return digitalRead(targetButtonID) == LOW ? PRESSED : NOT_PRESSED;
}

/*!
 * DIYPOWERMETERESP32::init
 *
 * initialization common to all consturctors
 *
 * @param[in] addr - device address
 *
 * @return success
*/
void DIYPOWERMETERESP32::init(uint8_t addr)
{
    /*** Initialize LED GPIO and State ***/
    pinMode(ledACT, OUTPUT);
    pinMode(ledNET, OUTPUT);
    pinMode(ledRLY, OUTPUT);

    this->setLED(LED_ACT, OFF);
    this->setLED(LED_NET, OFF);
    this->setLED(LED_RLY, OFF);

    /*** Initialize Relay GPIO ***/
    pinMode(relayLatchReset, OUTPUT);
    pinMode(relayLatchSet, OUTPUT);

    digitalWrite(relayLatchReset, HIGH);
    digitalWrite(relayLatchSet, HIGH);

    /*** Initialize Buzzer GPIO ***/
    pinMode(buzzerPin, OUTPUT);
    digitalWrite(buzzerPin, HIGH);

    /*** Initialize Button GPIO ***/
    pinMode(button1Pin, INPUT);
    pinMode(button2Pin, INPUT);

    /*** Check Power Meter Modbus Address ***/
    if (addr < 0x01)
    {
        addr = POWERMETER_DEFAULT_ADDRESS;
    }
    _addr = addr;

    // Set Initial lastRead time
    _lastRead = 0;
    _lastRead -= UPDATE_TIME;
}

/*!
 * DIYPOWERMETERESP32::updateValues
 *
 * Read all registers of device and update the local values
 *
 * @return success
*/
bool DIYPOWERMETERESP32::updateValues()
{
    static uint8_t response[40];
    uint16_t tempVar;

    // If we read before the update time limit, do not update
    if (_lastRead + UPDATE_TIME > millis())
    {
        return true;
    }

    // Read 8 register starting from 0x0048 (Voltage Register)
    tempVar = 0x0008;
    sendCmd(CMD_RHR, REG_VOLTAGE, &tempVar);

    if (receive(response, 37) != 37)
    { // Something went wrong
        return false;
    }

    /* Update the current values */
    _currentValues.voltage = ((uint32_t)response[3] << 24 | // Raw voltage in 0.1V
                              (uint32_t)response[4] << 16 |
                              (uint32_t)response[5] << 8 |
                              (uint32_t)response[6]) /
                             10000.0;

    _currentValues.current = ((uint32_t)response[7] << 24 | // Raw voltage in 0.1V
                              (uint32_t)response[8] << 16 |
                              (uint32_t)response[9] << 8 |
                              (uint32_t)response[10]) /
                             10000.0;

    _currentValues.power = ((uint32_t)response[11] << 24 | // Raw voltage in 0.1V
                            (uint32_t)response[12] << 16 |
                            (uint32_t)response[13] << 8 |
                            (uint32_t)response[14]) /
                           10000.0;

    _currentValues.energy = ((uint32_t)response[15] << 24 | // Raw voltage in 0.1V
                             (uint32_t)response[16] << 16 |
                             (uint32_t)response[17] << 8 |
                             (uint32_t)response[18]) /
                            10000.0;

    _currentValues.pf = ((uint32_t)response[19] << 24 | // Raw voltage in 0.1V
                         (uint32_t)response[20] << 16 |
                         (uint32_t)response[21] << 8 |
                         (uint32_t)response[22]) /
                        1000.0;

    _currentValues.co2 = ((uint32_t)response[23] << 24 | // Raw voltage in 0.1V
                          (uint32_t)response[24] << 16 |
                          (uint32_t)response[25] << 8 |
                          (uint32_t)response[26]) /
                         10000.0;

    _currentValues.coreTemperature = ((uint32_t)response[27] << 24 | // Raw voltage in 0.1V
                                      (uint32_t)response[28] << 16 |
                                      (uint32_t)response[29] << 8 |
                                      (uint32_t)response[30]) /
                                     100.0;

    _currentValues.frequency = ((uint32_t)response[31] << 24 | // Raw voltage in 0.1V
                                (uint32_t)response[32] << 16 |
                                (uint32_t)response[33] << 8 |
                                (uint32_t)response[34]) /
                               100.0;

    // Record current time as _lastRead
    _lastRead = millis();

    return true;
}

/*!
 * DIYPOWERMETERESP32::receive
 *
 * Receive data from serial with buffer limit and timeout
 *
 * @param[out] resp Memory buffer to hold response. Must be at least `len` long
 * @param[in] len Max number of bytes to read
 *
 * @return number of bytes read
*/
uint16_t DIYPOWERMETERESP32::receive(uint8_t *resp, uint16_t len, uint32_t timeout)
{
    unsigned long startTime = millis(); // Start time for Timeout
    uint8_t index = 0;                  // Bytes we have read

    while ((index < len) && (millis() - startTime < timeout))
    {
        if (_serial->available() > 0)
        {
            uint8_t c = (uint8_t)_serial->read();

            resp[index++] = c;
        }
        yield(); // do background netw tasks while blocked for IO (prevents ESP watchdog trigger)
    }

    Serial.print("Resp: ");
    for (int i = 0; i < index; i++)
    {
        if (resp[i] <= 0x0F)
            Serial.print("0");
        Serial.print(resp[i], HEX);
        Serial.print(" ");
    }
    Serial.println("END");

    // Check CRC with the number of bytes read
    if (!checkCRC(resp, index))
    {
        return 0;
    }

    return index;
}

/*!
 * DIYPOWERMETERESP32::checkCRC
 *
 * Performs CRC check of the buffer up to len-2 and compares check sum to last two bytes
 *
 * @param[in] data Memory buffer containing the frame to check
 * @param[in] len  Length of the respBuffer including 2 bytes for CRC
 *
 * @return is the buffer check sum valid
*/
bool DIYPOWERMETERESP32::checkCRC(const uint8_t *buf, uint16_t len)
{
    if (len <= 2) // Sanity check
        return false;

    uint16_t crc = CRC16(buf, len - 2); // Compute CRC of data
    return ((uint16_t)buf[len - 2] | (uint16_t)buf[len - 1] << 8) == crc;
}

/*!
 * DIYPOWERMETERESP32::setCRC
 *
 * Set last two bytes of buffer to CRC16 of the buffer up to byte len-2
 * Buffer must be able to hold at least 3 bytes
 *
 * @param[out] data Memory buffer containing the frame to checksum and write CRC to
 * @param[in] len  Length of the respBuffer including 2 bytes for CRC
 *
*/
void DIYPOWERMETERESP32::setCRC(uint8_t *buf, uint16_t len)
{
    if (len <= 2) // Sanity check
        return;

    uint16_t crc = CRC16(buf, len - 2); // CRC of data

    // Write high and low byte to last two positions
    buf[len - 2] = crc & 0xFF;        // Low byte first
    buf[len - 1] = (crc >> 8) & 0xFF; // High byte second
}

// Pre computed CRC table static
const uint16_t crcTable[] PROGMEM = {
    0X0000, 0XC0C1, 0XC181, 0X0140, 0XC301, 0X03C0, 0X0280, 0XC241,
    0XC601, 0X06C0, 0X0780, 0XC741, 0X0500, 0XC5C1, 0XC481, 0X0440,
    0XCC01, 0X0CC0, 0X0D80, 0XCD41, 0X0F00, 0XCFC1, 0XCE81, 0X0E40,
    0X0A00, 0XCAC1, 0XCB81, 0X0B40, 0XC901, 0X09C0, 0X0880, 0XC841,
    0XD801, 0X18C0, 0X1980, 0XD941, 0X1B00, 0XDBC1, 0XDA81, 0X1A40,
    0X1E00, 0XDEC1, 0XDF81, 0X1F40, 0XDD01, 0X1DC0, 0X1C80, 0XDC41,
    0X1400, 0XD4C1, 0XD581, 0X1540, 0XD701, 0X17C0, 0X1680, 0XD641,
    0XD201, 0X12C0, 0X1380, 0XD341, 0X1100, 0XD1C1, 0XD081, 0X1040,
    0XF001, 0X30C0, 0X3180, 0XF141, 0X3300, 0XF3C1, 0XF281, 0X3240,
    0X3600, 0XF6C1, 0XF781, 0X3740, 0XF501, 0X35C0, 0X3480, 0XF441,
    0X3C00, 0XFCC1, 0XFD81, 0X3D40, 0XFF01, 0X3FC0, 0X3E80, 0XFE41,
    0XFA01, 0X3AC0, 0X3B80, 0XFB41, 0X3900, 0XF9C1, 0XF881, 0X3840,
    0X2800, 0XE8C1, 0XE981, 0X2940, 0XEB01, 0X2BC0, 0X2A80, 0XEA41,
    0XEE01, 0X2EC0, 0X2F80, 0XEF41, 0X2D00, 0XEDC1, 0XEC81, 0X2C40,
    0XE401, 0X24C0, 0X2580, 0XE541, 0X2700, 0XE7C1, 0XE681, 0X2640,
    0X2200, 0XE2C1, 0XE381, 0X2340, 0XE101, 0X21C0, 0X2080, 0XE041,
    0XA001, 0X60C0, 0X6180, 0XA141, 0X6300, 0XA3C1, 0XA281, 0X6240,
    0X6600, 0XA6C1, 0XA781, 0X6740, 0XA501, 0X65C0, 0X6480, 0XA441,
    0X6C00, 0XACC1, 0XAD81, 0X6D40, 0XAF01, 0X6FC0, 0X6E80, 0XAE41,
    0XAA01, 0X6AC0, 0X6B80, 0XAB41, 0X6900, 0XA9C1, 0XA881, 0X6840,
    0X7800, 0XB8C1, 0XB981, 0X7940, 0XBB01, 0X7BC0, 0X7A80, 0XBA41,
    0XBE01, 0X7EC0, 0X7F80, 0XBF41, 0X7D00, 0XBDC1, 0XBC81, 0X7C40,
    0XB401, 0X74C0, 0X7580, 0XB541, 0X7700, 0XB7C1, 0XB681, 0X7640,
    0X7200, 0XB2C1, 0XB381, 0X7340, 0XB101, 0X71C0, 0X7080, 0XB041,
    0X5000, 0X90C1, 0X9181, 0X5140, 0X9301, 0X53C0, 0X5280, 0X9241,
    0X9601, 0X56C0, 0X5780, 0X9741, 0X5500, 0X95C1, 0X9481, 0X5440,
    0X9C01, 0X5CC0, 0X5D80, 0X9D41, 0X5F00, 0X9FC1, 0X9E81, 0X5E40,
    0X5A00, 0X9AC1, 0X9B81, 0X5B40, 0X9901, 0X59C0, 0X5880, 0X9841,
    0X8801, 0X48C0, 0X4980, 0X8941, 0X4B00, 0X8BC1, 0X8A81, 0X4A40,
    0X4E00, 0X8EC1, 0X8F81, 0X4F40, 0X8D01, 0X4DC0, 0X4C80, 0X8C41,
    0X4400, 0X84C1, 0X8581, 0X4540, 0X8701, 0X47C0, 0X4680, 0X8641,
    0X8201, 0X42C0, 0X4380, 0X8341, 0X4100, 0X81C1, 0X8081, 0X4040};

/*!
 * DIYPOWERMETERESP32::CRC16
 *
 * Calculate the CRC16-Modbus for a buffer
 * Based on https://www.modbustools.com/modbus_crc16.html
 *
 * @param[in] data Memory buffer containing the data to checksum
 * @param[in] len  Length of the respBuffer
 *
 * @return Calculated CRC
*/
uint16_t DIYPOWERMETERESP32::CRC16(const uint8_t *data, uint16_t len)
{
    uint8_t nTemp;         // CRC table index
    uint16_t crc = 0xFFFF; // Default value

    while (len--)
    {
        nTemp = *data++ ^ crc;
        crc >>= 8;
        crc ^= (uint16_t)pgm_read_word(&crcTable[nTemp]);
    }
    return crc;
}

bool DIYPOWERMETERESP32::sendCmd(uint8_t cmd, uint16_t regAddr, uint16_t *value, uint16_t dataLen, bool check, uint16_t slave_addr)
{
    uint16_t bufferIndex = 0;
    uint8_t sendBuffer[40]; // Send buffer
    uint8_t respBuffer[40]; // Response buffer (only used when check is true)

    if ((slave_addr == 0xFFFF) ||
        (slave_addr > 0xFF))
    {
        slave_addr = _addr;
    }

    sendBuffer[bufferIndex++] = slave_addr; // Set slave address
    sendBuffer[bufferIndex++] = cmd;        // Set command

    sendBuffer[bufferIndex++] = (regAddr >> 8) & 0xFF; // Set high byte of register address
    sendBuffer[bufferIndex++] = (regAddr)&0xFF;        // Set low byte =//=

    if (dataLen == 0)
    {
        sendBuffer[bufferIndex++] = (value[0] >> 8) & 0xFF; // Set high byte of register value
        sendBuffer[bufferIndex++] = (value[0]) & 0xFF;      // Set low byte =//=
    }
    else
    {
        sendBuffer[bufferIndex++] = (dataLen >> 8) & 0xFF; // Set high byte of register value
        sendBuffer[bufferIndex++] = (dataLen)&0xFF;        // Set low byte =//=

        sendBuffer[bufferIndex++] = dataLen * 2; // Number of Bytes

        for (int i = 0; i < dataLen; i++)
        {
            sendBuffer[bufferIndex++] = (value[i] >> 8) & 0xFF; // Set high byte of register value
            sendBuffer[bufferIndex++] = (value[i]) & 0xFF;      // Set low byte =//=
        }
    }
    bufferIndex += 2;
    setCRC(sendBuffer, bufferIndex); // Set CRC of frame

    delay(150);
    _serial->flush();
    _serial->write(sendBuffer, bufferIndex); // send frame

    Serial.print("\nSend: ");
    for (int x = 0; x < bufferIndex; x++)
    {
        if (sendBuffer[x] <= 0x0F)
            Serial.print("0");
        Serial.print(sendBuffer[x], HEX);
        Serial.print(" ");
    }

    Serial.println("END");

    if (check)
    {
        bufferIndex = receive(respBuffer, 8, WRITE_TIMEOUT); // Reply 8 Bytes
        if (!bufferIndex)
        { // if check enabled, read the response
            return false;
        }

        // Check Reply
        for (int i = 0; i < 6; i++)
        {
            if (i == 0 && slave_addr == 0)
            {
                i = 1;
            }
            if (sendBuffer[i] != respBuffer[i])
                return false;
        }
    }
    return true;
}

/*!
 * DIYPOWERMETERESP32::setAddress
 *
 * Set a new device address and update the device
 * WARNING - should be used to set up devices once.
 * Code initializtion will still have old address on next run!
 *
 * @param[in] addr New device address 0x01-0xFF
 *
 * @return success
*/
bool DIYPOWERMETERESP32::setConfig(uint8_t slaveAddr, enumPowerMeterConfigFrameFormat frameFormat, enumPowerMeterConfigBaudrate baudrate)
{
    uint16_t tempVar = 0;

#ifdef DEBUG
    Serial.print("\nSet Config...");
#endif

    if (slaveAddr < 0x01) // sanity check
        return false;

    if (frameFormat != POWERMETER_COMM_8N1 &&
        frameFormat != POWERMETER_COMM_8E1 &&
        frameFormat != POWERMETER_COMM_8O1 &&
        frameFormat != POWERMETER_COMM_8N2)
    {
#ifdef DEBUG
        Serial.println("FAILED. Invalid Frame Format.");
#endif
        return false;
    }

    if (baudrate != POWERMETER_BAUD_1200 &&
        baudrate != POWERMETER_BAUD_2400 &&
        baudrate != POWERMETER_BAUD_4800 &&
        baudrate != POWERMETER_BAUD_9600 &&
        baudrate != POWERMETER_BAUD_19200)
    {
#ifdef DEBUG
        Serial.println("FAILED. Invalid Baudrate.");
#endif
        return false;
    }

    tempVar = (slaveAddr << 8) | frameFormat | baudrate;

    // Write the new address to the address register
    if (!sendCmd(CMD_WMR, REG_CONFIG, &tempVar, 1, true, 0))
        return false;

    _addr = slaveAddr; // If successful, update the current slave address and settings value
    _commSettings = (frameFormat | baudrate);

#ifdef DEBUG
    Serial.print("Config: ");
    if (_addr <= 0x0F)
        Serial.print("0");
    Serial.print(_addr, HEX);
    Serial.print(",");
    Serial.print("0");
    Serial.println(_commSettings, HEX);
#endif
    return true;
}

/*!
 * DIYPOWERMETERESP32::getConfig
 *
 * Get the current device configuration
 * @return status (true: success, false: failed)
*/

bool DIYPOWERMETERESP32::getConfig()
{
    static uint8_t response[40];
    uint16_t tempVar;

#ifdef DEBUG
    Serial.print("\nGet Config...");
#endif

    // Read 8 register starting from 0x0048 (Voltage Register)
    tempVar = 0x0001;
    sendCmd(CMD_RHR, REG_CONFIG, &tempVar, 0, false, 0);

    if (receive(response, 7) != 7)
    { // Something went wrong
#ifdef DEBUG
        Serial.println("FAILED");
#endif
        return false;
    }

    /* Update the current values */
    _addr = response[3];         // Slave Address
    _commSettings = response[4]; // UART Frame Format (High Nibble) and BaudRate (Low Nibble)

#ifdef DEBUG
    Serial.print("Config: ");
    if (_addr <= 0x0F)
        Serial.print("0");
    Serial.print(_addr, HEX);
    Serial.print(",");
    Serial.print("0");
    Serial.println(_commSettings, HEX);
#endif

    return true;
}

/*!
 * DIYPOWERMETERESP32::checkConfig
 *
 * Check if current power meter module configuration is match with device preferred configuration values.
 * @return None
*/
void DIYPOWERMETERESP32::checkConfig()
{
    bool status = false;
    uint8_t checkSeq = 0;
#ifdef DEBUG
    Serial.print("Power Up Delay...");
#endif
    delay(2000);
#ifdef DEBUG
    Serial.print("Power Up Delay...DONE");
#endif

    while (1)
    {
        if (checkSeq == 2)
        {
            setBuzzer(ON);
            delay(200);
            setBuzzer(OFF);
            delay(200);
            yield();
        }
        else
        {
#ifdef DEBUG
            Serial.print("\ncheckSeq: ");
            Serial.println(checkSeq);
#endif
            if (checkSeq == 1)
            {
                updateInterfaceBaudrate(4800);
            }
            status = getConfig();
            if (status)
            {
                // Check if current device configuration match with default parameters. If not, update with current default values.
                if ((_addr != POWERMETER_DEFAULT_ADDRESS) || (_commSettings != (POWERMETER_DEFAULT_FRAME_FORMAT | POWERMETER_DEFAULT_BAUDRATE)))
                {
                    status = setConfig();
                    if (status)
                    {
                        if (checkSeq == 1)
                        {
                            updateInterfaceBaudrate(19200);
                            break;
                        }
                    }
                    else
                    {
#ifdef DEBUG
                        Serial.println("Set Config Failed... HALT!");
#endif
                        checkSeq = 2;
                    }
                }
                else
                {
                    break;
                }
            }
            else
            {
                if (checkSeq == 0)
                    checkSeq = 1;
                else
                {
#ifdef DEBUG
                    Serial.println("Retry Get Config Failed... HALT!");
#endif
                    checkSeq = 2;
                }
            }
        }
    }
}

/*!
 * DIYPOWERMETERESP32::getAddress
 *
 * Get the current device address
 *
 * @return address
*/
uint8_t DIYPOWERMETERESP32::getAddress()
{
    return _addr;
}

/*!
 * DIYPOWERMETERESP32::getCommSettings
 *
 * Get the current device communcation settings
 *
 * @return communication settings (UART Frame Format - High Nibble, UART Baud Rate - Low Nibble)
*/
uint8_t DIYPOWERMETERESP32::getCommSettings()
{
    return _commSettings;
}

/*!
 * DIYPOWERMETERESP32::resetEnergy
 *
 * Reset the Energy counter on the device
 *
 * @return success
*/
bool DIYPOWERMETERESP32::resetEnergy()
{
    uint16_t tempVar[2] = {0x0000, 0x0000};
#ifdef DEBUG
    Serial.print("\nReset Energy...");
#endif

    // Write 0x0000 to Register Energy (0x004B)
    if (!sendCmd(CMD_WMR, REG_ENERGY, tempVar, 2, true))
    {
#ifdef DEBUG
        Serial.println("FAILED");
        setBuzzer(ON);
        delay(1000);
        setBuzzer(OFF);
#endif
        return false;
    }

#ifdef DEBUG
    Serial.println("SUCCESS");
    setBuzzer(ON);
    delay(250);
    setBuzzer(OFF);
#endif
    return true;
}
