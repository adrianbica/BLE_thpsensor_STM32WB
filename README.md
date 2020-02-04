# BLE_thpsensor_STM32WB
BLE device based on STM32WB55 Nucleo Pack kit from STMicro

This is a custom profile with four services:
1. THP Sensing Service has three characteristics:
    a. Humidity characteristic
    b. Pressure characteristic
    c. Temperature characteristic
    
    The service is built around a BME280 sensor connected on the I2C bus to nRF52840-DK board
    
2. LED Service has one characteristic:
    a. Digital characteristic 
    
    This service is used to control the four LED's on the DK board
    
3. Relay Service has one characteristic:
    a. Digital characteristic
    
    This service is used to control four relays connected to the GPIO's on the DK board
    
4. ServoMotor Service has one characteristic:
    a. Analog characteristic
    
    This sevice is used to control the angle of a DS3218 servomotor    

Code was created with STM32CubeIDE v1.2.1 and STMCubeMX v5.5.0-RC6

The GATT Profile will look like in the following diagram:

![BLE_sensor_actuator](https://user-images.githubusercontent.com/4603301/71648796-04e61600-2cd7-11ea-985b-bb12b56dec18.png)
