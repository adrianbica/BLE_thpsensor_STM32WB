/*
 * thpsensor_stm.h
 *
 *  Created on: Jan 23, 2020
 *      Author: adrian
 */

#ifndef INC_THPSENSOR_STM_H_
#define INC_THPSENSOR_STM_H_

#ifdef __cplusplus
extern "C" {
#endif

void SVCCTL_BLEDeviceInitCustomSvc(void);
tBleStatus BLEDevice_STM_App_Update_Char(uint16_t UUID, uint8_t *pPayload);

#ifdef __cplusplus
}
#endif



#endif /* INC_THPSENSOR_STM_H_ */
