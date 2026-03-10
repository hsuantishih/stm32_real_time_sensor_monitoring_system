#include "adxl.h"
#include "main.h"
#include "myprintf.h"

extern I2C_HandleTypeDef hi2c1;

void Adxl_Write(uint8_t Reg, uint8_t Byte) {
    HAL_I2C_Mem_Write(&hi2c1, ADXL_ADDRESS, Reg, 1, &Byte, 1, 100);
}

void Adxl_Read(uint8_t Reg, uint8_t *Buffer, size_t len) {
    HAL_I2C_Mem_Read(&hi2c1, ADXL_ADDRESS, Reg, 1, Buffer, len, 100);
}

void Adxl_Init(void) {
    uint8_t chipID = 0;
    Adxl_Read(DEVICE_ID, &chipID, 1);

    if (chipID == 0xE5) {
        myprintf("ADXL345 found! Chip ID: %x\r\n", chipID);
        Adxl_Write(POWER_CTL, 0x00);
        Adxl_Write(POWER_CTL, 0x08);   // Set to measure mode
        Adxl_Write(DATA_FORMAT, 0x00); // Set full resolution, +/- 2g range
    } else {
        myprintf("ADXL345 not found! Chip ID: %x\r\n", chipID);
        Error_Handler();
    }
}

void Adxl_Read_Acceleration(float *ax, float *ay, float *az) {
    uint8_t data[6];
    Adxl_Read(DATAX0, data, 6);

    int16_t rawX = (data[1] << 8) | data[0];
    int16_t rawY = (data[3] << 8) | data[2];
    int16_t rawZ = (data[5] << 8) | data[4];

    *ax = (float)rawX * 0.003906f;
    *ay = (float)rawY * 0.003906f;
    *az = (float)rawZ * 0.003906f;
}