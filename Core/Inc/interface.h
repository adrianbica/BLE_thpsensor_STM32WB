/*
 * interface.h
 *
 *  Created on: Dec 1, 2019
 *      Author: adrian
 */

#ifndef INC_INTERFACE_H_
#define INC_INTERFACE_H_

#ifdef __cplusplus
extern "C" {
#endif


void Tick(void);
void InitSensorTask(I2C_HandleTypeDef *handle);
void RunSensorTask(void);
void SetTemperatureNotificationStatus(uint8_t status);
void SetPressureNotificationStatus(uint8_t status);
void SetHumidityNotificationStatus(uint8_t status);
void InitServoMotor(void);
void SetServoMotorPosition(uint16_t pos);
uint16_t GetServoMotorPosition(void);

#ifdef __cplusplus
}
#endif


#endif /* INC_INTERFACE_H_ */
