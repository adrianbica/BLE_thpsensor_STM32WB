/*
 * interface.cpp
 *
 *  Created on: Dec 1, 2019
 *      Author: adrian
 */

#include "timer.h"
#include "THPSensor.h"
#include "i2c_bus.h"
#include "ServoMotor.h"

#ifdef __cplusplus
extern "C" {
#endif


static THPSensor* 	thp_sensor;
static ServoMotor*	motor;

void Tick(void)
{
	Timer::GetInstance().Tick();
}

void InitSensorTask(I2C_HandleTypeDef *handle)
{
	thp_sensor = new THPSensor();
	i2c_bus::GetInstance().setHandle(handle);
	Timer::GetInstance().ArmTimer(SENSOR_TIMER);
}

void RunSensorTask(void)
{
	thp_sensor->task();
}

void SetTemperatureNotificationStatus(uint8_t status)
{
	thp_sensor->setTemperatureNotifyStatus(status);
}

void SetPressureNotificationStatus(uint8_t status)
{
	thp_sensor->setPressureNotifyStatus(status);
}

void SetHumidityNotificationStatus(uint8_t status)
{
	thp_sensor->setHumidityNotifyStatus(status);
}

void InitServoMotor(void)
{
	motor = new ServoMotor(0, 0xffff, 0x7fff, 430, 2560, 1492);
}

void SetServoMotorPosition(uint16_t pos)
{
	motor->SetCurrValue(pos);
}

uint16_t GetServoMotorPosition(void)
{
	return motor->GetCurrValue();
}

#ifdef __cplusplus
}
#endif

