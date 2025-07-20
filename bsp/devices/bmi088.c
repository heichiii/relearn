//
// Created by GaoKailong on 25-7-16.
//

#include "bmi088.h"
#include "spi.h"
#include "stm32h7xx_hal_gpio.h"
#include <math.h>
#define START_ACC HAL_GPIO_WritePin(CSB1_ACC_GPIO_Port, CSB1_ACC_Pin, GPIO_PIN_RESET)
#define START_GYR HAL_GPIO_WritePin(CSB2_GYR_GPIO_Port, CSB2_GYR_Pin, GPIO_PIN_RESET)
#define STOP_ACC HAL_GPIO_WritePin(CSB1_ACC_GPIO_Port, CSB1_ACC_Pin, GPIO_PIN_SET)
#define STOP_GYR HAL_GPIO_WritePin(CSB2_GYR_GPIO_Port, CSB2_GYR_Pin, GPIO_PIN_SET)

// static function prototypes
static void  readAccReg(uint8_t reg, uint8_t *data);
static void  readGyroReg(uint8_t reg, uint8_t *data);
static void  writeAccReg(uint8_t reg, uint8_t data);
static void  writeGyroReg(uint8_t reg, uint8_t data);
static void  configBMI088(BMI088 *dev);

// user interface functions
void initBMI088(BMI088 *dev)
{

    while (dev->gyro_id!=0x0F)
    {
        readGyroReg(BMI08_REG_GYRO_CHIP_ID, &dev->gyro_id);
        HAL_Delay(1);
    }
    while (dev->acc_id!=0x1E)
    {
        readAccReg(BMI08_REG_ACCEL_CHIP_ID, &dev->acc_id);
        HAL_Delay(1);
    }

    configBMI088(dev);


}

void updateBMI088(BMI088 *dev)
{
    uint8_t acc_range;
    //read
    //todo
    readAccReg(BMI08_REG_ACCEL_RANGE, &acc_range);
    readAccReg(BMI08_REG_ACCEL_X_LSB, dev->acc_x_temp);
    readAccReg(BMI08_REG_ACCEL_X_MSB, dev->acc_x_temp+1);
    readAccReg(BMI08_REG_ACCEL_Y_LSB, dev->acc_y_temp);
    readAccReg(BMI08_REG_ACCEL_Y_MSB, dev->acc_y_temp+1);
    readAccReg(BMI08_REG_ACCEL_Z_LSB, dev->acc_z_temp);
    readAccReg(BMI08_REG_ACCEL_Z_MSB, dev->acc_z_temp+1);
    // readAccReg(BMI08_REG_ACCEL_TEMP_LSB, dev->acc_temperature_temp);
    // readAccReg(BMI08_REG_ACCEL_TEMP_MSB, dev->acc_temperature_temp+1);
    readGyroReg(BMI08_REG_GYRO_X_LSB, dev->rate_x_temp);
    readGyroReg(BMI08_REG_GYRO_X_MSB, dev->rate_x_temp+1);
    readGyroReg(BMI08_REG_GYRO_Y_LSB, dev->rate_y_temp);
    readGyroReg(BMI08_REG_GYRO_Y_MSB, dev->rate_y_temp+1);
    readGyroReg(BMI08_REG_GYRO_Z_LSB, dev->rate_z_temp);
    readGyroReg(BMI08_REG_GYRO_Z_MSB, dev->rate_z_temp+1);


    dev->acc_x = (int16_t)(dev->acc_x_temp[1]<<8 | dev->acc_x_temp[0])/32768.0f*/*1000.0f**/(float)(pow(2,(acc_range+1)))*1.5f*9.8f; // 2^(-15) * 1000
    dev->acc_y = (int16_t)(dev->acc_y_temp[1]<<8 | dev->acc_y_temp[0])/32768.0f*/*1000.0f**/(float)(pow(2,(acc_range+1)))*1.5f*9.8f;
    dev->acc_z = (int16_t)(dev->acc_z_temp[1]<<8 | dev->acc_z_temp[0])/32768.0f*/*1000.0f**/(float)(pow(2,(acc_range+1)))*1.5f*9.8f;
    dev->rate_x = (int16_t)(dev->rate_x_temp[1]<<8 | dev->rate_x_temp[0])*30.5f/1000.0f;
    dev->rate_y = (int16_t)(dev->rate_y_temp[1]<<8 | dev->rate_y_temp[0])*30.5f/1000.0f;
    dev->rate_z = (int16_t)(dev->rate_z_temp[1]<<8 | dev->rate_z_temp[0])*30.5f/1000.0f;
}




void readAccReg(uint8_t reg, uint8_t *data) {
    uint8_t tx_buf[2] = {reg | 0x80, 0x00}; // 命令 + 哑字节（任意值）
    uint8_t rx_buf[2];

    START_ACC;
    HAL_SPI_TransmitReceive(&hspi2, tx_buf, rx_buf, 1, 1000); // 阻塞式收发
    HAL_SPI_TransmitReceive(&hspi2, tx_buf+1, rx_buf, 1, 1000);
    HAL_SPI_TransmitReceive(&hspi2, tx_buf+1, rx_buf+1, 1, 1000);
    STOP_ACC;

    *data = rx_buf[1]; // 丢弃rx_buf[0]（哑字节），取真实数据
}
void readGyroReg(uint8_t reg, uint8_t *data)
{
    uint8_t tx_buf[1] = {reg | 0x80};
    START_GYR;
    HAL_SPI_TransmitReceive(&hspi2, tx_buf, data, 1, 100); // 阻塞式收发
    HAL_SPI_TransmitReceive(&hspi2, tx_buf, data, 1, 100); // 阻塞式收发
    STOP_GYR;
}
void writeAccReg(uint8_t reg, uint8_t data)
{
    uint8_t tx_buf[2] = {reg & 0x7F, data};
    uint8_t rx_buf[1];
    START_ACC;
    HAL_SPI_TransmitReceive(&hspi2, tx_buf, rx_buf, 2, 100); // 阻塞式收发
    STOP_ACC;
}
static void writeGyroReg(uint8_t reg, uint8_t data)
{
    uint8_t tx_buf[2] = {reg & 0x7F, data};
    uint8_t rx_buf[1];
    START_GYR;
    HAL_SPI_TransmitReceive(&hspi2, tx_buf, rx_buf, 2, 100); // 阻塞式收发
    STOP_GYR;
}

void configBMI088(BMI088 *dev)
{

    writeAccReg(BMI08_REG_ACCEL_PWR_CTRL,0x04);
    writeAccReg(BMI08_REG_ACCEL_PWR_CONF,0x00);

    writeAccReg(BMI08_REG_ACCEL_RANGE, 0x00); // 设置加速度计范围
    writeGyroReg(BMI08_REG_GYRO_RANGE, 0x01); // 设置陀螺仪范围
}
