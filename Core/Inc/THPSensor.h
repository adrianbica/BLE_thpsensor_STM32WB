#ifndef __THP_SENSOR_H__
#define __THP_SENSOR_H__

#include <list>
#include "sensor.h"
//#include "thp_sensing_service.h"
//#include "ble/HumidityCharacteristic.h"
//#include "ble/PressureCharacteristic.h"
//#include "ble/TemperatureCharacteristic.h"

using namespace std;

class CALIBRATION {
public:
    uint16_t        dig_T1;
    int16_t         dig_T2;
    int16_t         dig_T3;
    uint16_t        dig_P1;
    int16_t         dig_P2;
    int16_t         dig_P3;
    int16_t         dig_P4;
    int16_t         dig_P5;
    int16_t         dig_P6;
    int16_t         dig_P7;
    int16_t         dig_P8;
    int16_t         dig_P9;
    uint8_t         dig_H1;
    int16_t         dig_H2;
    uint8_t         dig_H3;
    int16_t         dig_H4;
    int16_t         dig_H5;
    uint8_t         dig_H6;
};

/*typedef void(*callback)(void);

typedef struct tagStateMachine {
    SENSOR_STATE    state;
    callback        clbk;
} StateMachineEntry;*/



class THPSensor: public sensor
{
private:
    int16_t         Temperature;
    uint32_t        Pressure;
    uint16_t        Humidity;

    uint8_t			TemperatureNotifyStatus;
    uint8_t			PressureNotifyStatus;
    uint8_t			HumidityNotifyStatus;

//    SensingServiceHumidity      H;
//    SensingServicePressure      P;
//    SensingServiceTemperature   T;
    //thp_sensing_service_t               *Service;
    CALIBRATION     Calibr;
    //list<StateMachineEntry>    sm;
    void checkAnswerTimeout(void);
    void calibrate(void);
    void configure(void);
    void measure(void);
public:
    void init(void);
    THPSensor(void);
    //THPSensor(ble_thp_sensing_service_t *service) {Service = service;};
    void task(void);
    void setTemperatureNotifyStatus(uint8_t v) { TemperatureNotifyStatus = v; }
    void setPressureNotifyStatus(uint8_t v) { PressureNotifyStatus = v; }
    void setHumidityNotifyStatus(uint8_t v) { HumidityNotifyStatus = v; }
};

#endif

