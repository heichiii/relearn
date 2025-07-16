//
// Created by GaoKailong on 25-7-16.
//

#ifndef BMI088_H
#define BMI088_H
#include "bmi08_defs.h"
#include "stm32h7xx_hal_gpio.h"
#define START_ACC HAL_GPIO_WritePin(CSB1_ACC_GPIO_Port, CSB1_ACC_Pin, GPIO_PIN_RESET)
#define START_GYR HAL_GPIO_WritePin(CSB2_GYR_GPIO_Port, CSB2_GYR_Pin, GPIO_PIN_RESET)
#define STOP_ACC HAL_GPIO_WritePin(CSB1_ACC_GPIO_Port, CSB1_ACC_Pin, GPIO_PIN_SET)
#define STOP_GYR HAL_GPIO_WritePin(CSB2_GYR_GPIO_Port, CSB2_GYR_Pin, GPIO_PIN_SET)

typedef struct
{
	uint8_t acc_id;
	uint8_t gyro_id;
	/*
	The unit is in LSB. The conversion from LSB to acceleration (mg) is based on the range settings and
	can be calculated as follows (<0x41>: content of the ACC_RANGE register):
	Accel_X_in_mg = Accel_X_int16 / 32768 * 1000 * 2^(<0x41> + 1) * 1.5
	Accel_Y_in_mg = Accel_Y_int16 / 32768 * 1000 * 2^(<0x41> + 1) * 1.5
	Accel_Z_in_mg = Accel_Z_int16 / 32768 * 1000 * 2^(<0x41> + 1) * 1.5
	*/
	float acc_x;
	float acc_y;
	float acc_z;
	uint8_t acc_x_temp[2];
	uint8_t acc_y_temp[2];
	uint8_t acc_z_temp[2];


	/*
	The data is stored in an 11-bit value in 2’s
	complement format. The resolution is 0.125°C/LSB, thus the temperature can be obtained as follows:
	Temp_uint11 = (TEMP_MSB * 8) + (TEMP_LSB / 32)
	if Temp_uint11 > 1023:
	Temp_int11 = Temp_uint11 – 2048
	else:
	Temp_int11 = Temp_uint11
	Temperature = Temp_int11 * 0,125°C/LSB + 23°C
	*/
	float acc_temperature;
	uint8_t acc_temperature_temp[2];

	/*
	From the registers, the gyro values can be
	calculated as follows:
	Rate_X: RATE_X_MSB * 256 + RATE_X_LSB
	Rate_Y: RATE_Y_MSB * 256 + RATE_Y_LSB
	Rate_Z: RATE_Z_MSB * 256 + RATE_Z_LSB
	 */
	float rate_x;
	float rate_y;
	float rate_z;
	uint8_t rate_x_temp[2];
	uint8_t rate_y_temp[2];
	uint8_t rate_z_temp[2];

}BMI08;

void initBMI08(BMI08 *dev);
void getAccelData(BMI08 *dev);
void getGyroData(BMI08 *dev);

#endif //BMI088_H
