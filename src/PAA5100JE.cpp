#include "PAA5100JE.h"

PAA5100JE_OF::PAA5100JE_OF(uint8_t CS_PIN, SPIClass *spi) : _CS_PIN(CS_PIN)
{
    _spi = spi;
}

PAA5100JE_OF::~PAA5100JE_OF()
{
    digitalWrite(_CS_PIN, HIGH);
}

// Driver initialization routine.
bool PAA5100JE_OF::init()
{
    _sensorSettings = SPISettings(2000000, MSBFIRST, SPI_MODE3);

    pinMode(_CS_PIN, OUTPUT);

    delay(1);
    _spi->beginTransaction(_sensorSettings);
    digitalWrite(_CS_PIN, HIGH);
    delay(1);
    digitalWrite(_CS_PIN, LOW);
    delay(1);
    digitalWrite(_CS_PIN, HIGH);
    _spi->endTransaction();
    delay(1);
    if (powerUpSeq() == false)
        return false;
    else
        return true;
}

// Power up method checks product/inv_product ID registers for sanity check
// and does the performance optimization routine.
bool PAA5100JE_OF::powerUpSeq()
{
    writeRegister(POWER_UP_RESET_REG, 0x5A);
    delay(10);

    uint8_t productID = readRegister(PRODUCT_ID_REG);
    uint8_t invProductID = readRegister(INVERSE_PRODUCT_ID_REG);
    if (productID != 0x49 && invProductID != 0xB6)
        return false;

    readRegister(MOTION_REG);
    readRegister(DELTA_X_L_REG);
    readRegister(DELTA_X_H_REG);
    readRegister(DELTA_Y_L_REG);
    readRegister(DELTA_Y_H_REG);
    readRegister(DELTA_X_H_REG);

    delay(1);

    performanceOptimizationRoutine();

    _shutdown_flag = false;

    return true;
}

// Power up method, setts sensor in shutdown mode.
bool PAA5100JE_OF::powerDownSeq()
{
    writeRegister(SHUTDOWN_REG, 0xB6);
    _shutdown_flag = true;
    return true;
}

// Method for setting the working height and CPI (counts per inch).
// Must be set before using getDistance() method.
// Height is in mm, must be as precise as possible, as little
// variations in height make big differences.
void PAA5100JE_OF::setWorkingHeight(double height)
{
    if (_shutdown_flag == true)
        return;
    if (height < 12.5 || height > 37.5)
        return;

    height /= 1000; // from mm to m
    // PixArt approximation formula from height to CPI
    _CPI = 11.914 * (1 / (height));
    _CPIset_flag = true;
}

// Another approach to setting CPI, using formula provided in README.
void PAA5100JE_OF::setCPI(double CPI)
{
    _CPI = CPI;
}

// Method for setting sensor resolution
// Value must be [0,168].
// Method is not being used anywhere, as defalut
// value of 168 is used.
void PAA5100JE_OF::setResolution(uint8_t resolution)
{
    if (resolution >= 0xA8)
        resolution = 0xA8;
    if (resolution < 0)
        resolution = 0;
    writeRegister(RESOLUTION_REG, resolution);
}

// Method for setting X/Y axis orientation.
void PAA5100JE_OF::setOrientation(bool invertX, bool invertY, bool swapXandY)
{
    if (_shutdown_flag == true)
        return;
    uint8_t sendVal = 0b0;
    if (invertX == true)
        sendVal |= 0b00100000;
    if (invertY == true)
        sendVal |= 0b01000000;
    if (swapXandY == true)
        sendVal |= 0b10000000;
    writeRegister(ORIENTATION_REG, sendVal);
}

// Method for getting surface quality info.
int PAA5100JE_OF::getSqual()
{
    if (_shutdown_flag == true)
        return -99;
    else
        return int(readRegister(SQUAL_REG)) * 4;
}

uint8_t PAA5100JE_OF::getObservation()
{
    return readRegister(OBSERVATION_REG);
}

// Motion read using normal read mode. Returns values in mm.
// CPI must be defined, for a method to work, as CPI depends
// on working height.
bool PAA5100JE_OF::getDistance(double deltaVal[])
{
    if (_shutdown_flag == true || _CPIset_flag == false)
        return false;
    int16_t tempDeltaVal[2] = {0, 0};
    if (motionRead(tempDeltaVal) == false)
        return false;
    deltaVal[0] = double(tempDeltaVal[0]) * (INCH_TO_MM / _CPI);
    deltaVal[1] = double(tempDeltaVal[1]) * (INCH_TO_MM / _CPI);
    return true;
}

// Motion read using burst read mode. Returns values in mm.
// CPI must be defined, for a method to work, as CPI depends
// on working height.
bool PAA5100JE_OF::getDistance_burst(double deltaVal[])
{
    if (_shutdown_flag == true || _CPIset_flag == false)
        return false;
    int16_t tempDeltaVal[2] = {0, 0};
    if (burstMotionRead(tempDeltaVal) == false)
        return false;
    deltaVal[0] = tempDeltaVal[0] * (1 / _CPI) * INCH_TO_MM;
    deltaVal[1] = tempDeltaVal[1] * (1 / _CPI) * INCH_TO_MM;
    return true;
}

// Method used for motion read.
// Returns values in counts.
bool PAA5100JE_OF::motionRead(int16_t deltaVal[])
{
    if (_shutdown_flag == true)
        return false;
    uint8_t motionOccur = readRegister(MOTION_REG);
    motionOccur &= 0x80u;
    if (motionOccur != 0x80u)
    {
        deltaVal[0] = 0;
        deltaVal[1] = 0;
        return true;
    }
    else
    {
        uint8_t tempX_L = readRegister(DELTA_X_L_REG);
        uint8_t tempX_H = readRegister(DELTA_X_H_REG);
        uint8_t tempY_L = readRegister(DELTA_Y_L_REG);
        uint8_t tempY_H = readRegister(DELTA_Y_H_REG);
        uint8_t squal = readRegister(SQUAL_REG);
        uint8_t shutter_upper = readRegister(SHUTTER_LOWER_REG);
        if (squal < 0x19 || shutter_upper == 0x1F)
        {
            return false;
        }
        else
        {
            deltaVal[0] = ((int16_t)tempX_H << 8) | tempX_L;
            deltaVal[1] = ((int16_t)tempY_H << 8) | tempY_L;
            return true;
        }
    }
}

// Method used for burst motion read (no delay between reigister reads,
// faster implementation than normalRead()).
// Returns values in counts.
bool PAA5100JE_OF::burstMotionRead(int16_t deltaVal[])
{
    if (_shutdown_flag == true)
        return false;

    int16_t tempBurstRet[12];
    uint8_t regAdr = MOTION_BURST_REG;
    regAdr &= ~0x80u;

    _spi->beginTransaction(_sensorSettings);
    digitalWrite(_CS_PIN, LOW);
    delayMicroseconds(1);
    _spi->transfer(regAdr);
    delayMicroseconds(5);
    for (int i = 0; i < 12; i++)
    {
        tempBurstRet[i] = _spi->transfer(0x00);
    }
    delayMicroseconds(5);

    digitalWrite(_CS_PIN, HIGH);
    _spi->endTransaction();

    if (tempBurstRet[6] < 0x19 || tempBurstRet[10] == 0x1F)
    {
        return false;
    }
    else
    {
        deltaVal[0] = ((int16_t)tempBurstRet[3] << 8) | tempBurstRet[2];
        deltaVal[1] = ((int16_t)tempBurstRet[5] << 8) | tempBurstRet[4];
        return true;
    }
}

void PAA5100JE_OF::frameSync_stopOperation()
{
    for (int i = 0; i < 8; i++)
    {
        writeRegister(writeFrameSyncAdr_1[i], writeFrameSyncAdr_1[i]);
    }
}

// Helping method used to write to register
void PAA5100JE_OF::writeRegister(uint8_t regAdr, uint8_t dataOut)
{
    regAdr |= 0x80u;
    _spi->beginTransaction(_sensorSettings);
    digitalWrite(_CS_PIN, LOW);
    delayMicroseconds(1);
    _spi->transfer(regAdr);
    _spi->transfer(dataOut);
    delayMicroseconds(5);
    digitalWrite(_CS_PIN, HIGH);
    _spi->endTransaction();
    delayMicroseconds(5);
}

// Helping method used to read from register
uint8_t PAA5100JE_OF::readRegister(uint8_t regAdr)
{
    regAdr &= ~0x80u;
    _spi->beginTransaction(_sensorSettings);
    digitalWrite(_CS_PIN, LOW);
    delayMicroseconds(1);
    _spi->transfer(regAdr);
    delayMicroseconds(5);
    uint8_t dataIn = _spi->transfer(0x00);
    delayMicroseconds(1);
    digitalWrite(_CS_PIN, HIGH);
    _spi->endTransaction();
    delayMicroseconds(5);
    return dataIn;
}

// Method used at every startup to help with sending performance optimization registers.
// This registers are propiretery to PixArt ("magic registers").
void PAA5100JE_OF::performanceOptimizationRoutine()
{
    uint8_t tempRetVal = 0;
    int C1 = 0, C2 = 0;
    for (int i = 0; i < 5; i++)
    {
        writeRegister(writeRegAdrOptSend_1[i], writeValueOptSend_1[i]);
    }
    tempRetVal = readRegister(0x67);
    tempRetVal = tempRetVal & 0x80u;
    if (tempRetVal == 0x80u)
        writeRegister(0x48, 0x04);
    else
        writeRegister(0x48, 0x02);
    for (int i = 0; i < 5; i++)
    {
        writeRegister(writeRegAdrOptSend_2[i], writeValueOptSend_2[i]);
    }
    if (readRegister(0x73) == 0x00)
    {
        C1 = int(readRegister(0x70));
        if (C1 <= 28)
            C1 += 14;
        else
            C1 += 11;
        if (C1 > 63)
            C1 = 63;
        C2 = int(readRegister(0x71));
        C2 = (C2 * 45) / 100;
        writeRegister(0x7F, 0x00);
        writeRegister(0x61, 0xAD);
        writeRegister(0x51, 0x70);
        writeRegister(0x7F, 0x0E);

        writeRegister(0x70, uint8_t(C1));
        writeRegister(0x71, uint8_t(C2));
    }
    for (int i = 0; i < 67; i++)
    {
        writeRegister(writeRegAdrOptSend_3[i], writeValueOptSend_3[i]);
    }
    delay(10);
    for (int i = 0; i < 16; i++)
    {
        writeRegister(writeRegAdrOptSend_4[i], writeValueOptSend_4[i]);
    }
    delay(10);
    writeRegister(0x73, 0x00);
}
