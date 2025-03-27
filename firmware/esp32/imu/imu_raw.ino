/*
*
* Example for ICM-20948 (I2C), ESP32 Pinout
*   - SDA -> GPIO 21    (data line)
*   - SCL -> GPIO 22    (clock line)
*   - VCC -> 3.3V       (power supply)
*   - GND -> GND        (ground)
*
*/

#include <Wire.h>

// IMU I2C address (0x68 if AD0=GND, 0x69 if AD0=VCC)
#define ICM20948_I2C_ADDRESS 0x68

/* Registers */

// Many devices organize their registers into bank to maximize
// the number of configuration and data registers available.
#define REG_BANK_SEL    0x7F

// Bank 0
#define WHO_AM_I        0x00
#define PWR_MGMT_1      0x06
#define INT_PIN_CFG     0x0F
#define ACCEL_XOUT_H    0x2D
#define ACCEL_XOUT_L    0x2E
#define ACCEL_YOUT_H    0x2F
#define ACCEL_YOUT_L    0x30
#define ACCEL_ZOUT_H    0x31
#define ACCEL_ZOUT_L    0x32
#define GYRO_XOUT_H     0x33
#define GYRO_XOUT_L     0x34
#define GYRO_YOUT_H     0x35
#define GYRO_YOUT_L     0x36
#define GYRO_ZOUT_H     0x37
#define GYRO_ZOUT_L     0x38

// Bank 2
#define GYRO_CONFIG_1   0x01
#define ACCEL_CONFIG    0x14

// WHO_AM_I default value for ICM-20948
#define WHO_AM_I_VAL 0xEA

// Functions for writing to registers (in current bank)
void writeRegister(uint8_t reg, uint8_t value)
{
    Wire.beginTransmission(ICM20948_I2C_ADDRESS);
    Wire.write(reg);
    Wire.write(value);
    Wire.endTransmission();
}

// Function for reading one byte from register (in current bank)
uint8_t readRegister(uint8_t reg)
{
    Wire.beginTransmission(ICM20948_I2C_ADDRESS);
    Wire.write(reg);
    Wire.endTransmission(false);

    Wire.requestFrom(ICM20948_I2C_ADDRESS, (uint8_t)1);
    if (Wire.available())
    {
        return Wire.read();
    }

    return 0xFF; // error while reading
}

// Function for selecting registers bank
void selectBank(uint8_t bank)
{
    // Bank is fitting in [4:0] bits, so there's no need to shift for 4 bits left,
    // because ICM-20948 datasheet shows that BNK of selector is [2:0].
    // For most datasheets BANK = (bank << 4).
    uint8_t bankRegVal = (bank << 4);
    writeRegister(REG_BANK_SEL, bankRegVal);
}

// Function for ICM-20948 reset & initialization
bool initICM20948()
{
    // Module reset - writing 1 in DEVICE_RESET bit for PWR_MGMT_1 register
    selectBank(0);
    writeRegister(PWR_MGMT_1, 0x80); // DEVICE_RESET = 1
    delay(100);

    // Checking WHO_AM_I
    uint8_t whoAmI = readRegister(WHO_AM_I);
    if (whoAmI != WHO_AM_I_VAL)
    {
        Serial.print("Error: WHO_AM_I is waiting for 0x");
        Serial.print(WHO_AM_I_VAL, HEX);
        Serial.print(", got 0x");
        Serial.println(whoAmI, HEX);
        return false;
    }

    // Printing out message about success
    Serial.print("ICM-20948 was found, WHO_AM_I = 0x");
    Serial.println(whoAmI, HEX);

    // We need to clear SLEEP bit in PWR_MGMT_1 to remove module from sleep mode
    // And we also need to set auto clock selection (Quartz/PPL)
    writeRegister(PWR_MGMT_1, 0x01);
    delay(10);

    // Setting up gyroscope and accelerometer
    // Selecting Bank 2 to access GYRO_CONFIG_1, ACCEL_CONFIG etc.
    selectBank(2);

    // GYRO_CONFIG_1:
    // [7:6] = 00 (2000 dps), [5:4] = 00 (filter), [3:0] = FCHOICE
    // Let's set +-2000 dps for example, without filters (main mode)
    writeRegister(GYRO_CONFIG_1, 0x00);
    
    // ACCEL_CONFIG:
    // [7:6] = 00 (+-2g), [5:4] = 00 (filter), [3:0] = ACCEL_FCHOICE
    // Let's set +-2g, without extra filter
    writeRegister(ACCEL_CONFIG, 0x00);

    // We need to select Bank 0 again for reading data
    selectBank(0);

    return true;
}

// Function for reading raw 16-bit value: two bytes (High, Low) with addition to 2
int16_t readRawData(uint8_t regHigh, uint8_t regLow)
{
    int16_t highByte    = (int16_t)readRegister(regHigh);
    int16_t lowByte     = (int16_t)readRegister(regLow);
    return (int16_t)((highByte << 8) | lowByte);
}

// Main setup function
void setup()
{
    Serial.begin(115200);
    Wire.begin(21, 22, 400000); // 400 kHz

    delay(100);

    Serial.println("ICM-20948 initialization...");

    if (!initICM20948())
    {
        Serial.println("ICM-20948 was not found!");
        while (1)
        {
            // Infinite loop if sensor was not found
        }
    }

    Serial.println("ICM-20948 was successfully initialized.");
}

// Main loop function
void loop()
{
    // Reading raw data from accelerometer
    int16_t accelX = readRawData(ACCEL_XOUT_H, ACCEL_XOUT_L);
    int16_t accelY = readRawData(ACCEL_YOUT_H, ACCEL_YOUT_L);
    int16_t accelZ = readRawData(ACCEL_ZOUT_H, ACCEL_ZOUT_L);

    // Reading raw data from gyroscope
    int16_t gyroX = readRawData(GYRO_XOUT_H, GYRO_XOUT_L);
    int16_t gyroY = readRawData(GYRO_YOUT_H, GYRO_YOUT_L);
    int16_t gyroZ = readRawData(GYRO_ZOUT_H, GYRO_ZOUT_L);

    // Transforming to physical values
    // For current diapason of accelerometer +-2g = 16384 LSB/g
    float aRes = 1.0 / 16384.0;
    float Ax = accelX * aRes;
    float Ay = accelY * aRes;
    float Az = accelZ * aRes;

    // For gyroscope if diapason is +-2000 dps, then sensitivity is ~16.4 LSB/°/s
    float gRes = 1.0 / 16.4;
    float Gx = gyroX * gRes;
    float Gy = gyroY * gRes;
    float Gz = gyroZ * gRes;

    // Printing to Serial
    Serial.print("Accel (g): X=");
    Serial.print(Ax, 3);
    Serial.print(" Y=");
    Serial.print(Ay, 3);
    Serial.print(" Z=");
    Serial.print(Az, 3);
    Serial.print(" | Gyro (dps): X=");
    Serial.print(Gx, 3);
    Serial.print(" Y=");
    Serial.print(Gy, 3);
    Serial.print(" Z=");
    Serial.println(Gz, 3);

    delay(100); // 10 Hz
}