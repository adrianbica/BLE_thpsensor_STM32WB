/*
 * thpsensor_stm.c
 *
 *  Created on: Jan 23, 2020
 *      Author: adrian
 */

/* Includes ------------------------------------------------------------------*/
#include "common_blesvc.h"

/* Private typedef -----------------------------------------------------------*/
typedef struct{
	uint16_t	THPSensorSvcHdle;				/**< Service handle */
	uint16_t	HumidityCharHdle;	  			/**< Characteristic handle */
	uint16_t	PressureCharHdle;	  			/**< Characteristic handle */
	uint16_t	TemperatureCharHdle;	  			/**< Characteristic handle */

	uint16_t	LedSvcHdle;				/**< Service handle */
	uint16_t	LedDigitalCharHdle;	  			/**< Characteristic handle */

	uint16_t	RelaySvcHdle;				/**< Service handle */
	uint16_t	RelayDigitalCharHdle;	  			/**< Characteristic handle */

	uint16_t	ServoSvcHdle;				/**< Service handle */
	uint16_t	ServoAnalogCharHdle;	  			/**< Characteristic handle */
} BLEDEviceContext_t;

/* Private defines -----------------------------------------------------------*/
#define UUID_128_SUPPORTED  1

#if (UUID_128_SUPPORTED == 1)
#define BM_UUID_LENGTH  UUID_TYPE_128
#else
#define BM_UUID_LENGTH  UUID_TYPE_16
#endif

#define BM_REQ_CHAR_SIZE    (3)


/* Private macros ------------------------------------------------------------*/

/* Private variables ---------------------------------------------------------*/
PLACE_IN_SECTION("BLE_DRIVER_CONTEXT") static BLEDEviceContext_t aBLEDeviceContext;


/* Private function prototypes -----------------------------------------------*/
static SVCCTL_EvtAckStatus_t BLEDevice_Event_Handler(void *pckt);


/* Functions Definition ------------------------------------------------------*/
/* Private functions ----------------------------------------------------------*/

#define COPY_UUID_128(uuid_struct, uuid_15, uuid_14, uuid_13, uuid_12, uuid_11, uuid_10, uuid_9, uuid_8, uuid_7, uuid_6, uuid_5, uuid_4, uuid_3, uuid_2, uuid_1, uuid_0) \
		do {\
			uuid_struct[0] = uuid_0; uuid_struct[1] = uuid_1; uuid_struct[2] = uuid_2; uuid_struct[3] = uuid_3; \
			uuid_struct[4] = uuid_4; uuid_struct[5] = uuid_5; uuid_struct[6] = uuid_6; uuid_struct[7] = uuid_7; \
			uuid_struct[8] = uuid_8; uuid_struct[9] = uuid_9; uuid_struct[10] = uuid_10; uuid_struct[11] = uuid_11; \
			uuid_struct[12] = uuid_12; uuid_struct[13] = uuid_13; uuid_struct[14] = uuid_14; uuid_struct[15] = uuid_15; \
		}while(0)

#define COPY_THPSENSOR_SERVICE_UUID(uuid_struct)      COPY_UUID_128(uuid_struct,0xb2,0xb7,0x00,0x00,0x00,0x01,0x4c,0xb2,0xb3,0x4a,0x65,0x50,0xcc,0x0e,0x99,0x8c)
#define COPY_LED_SERVICE_UUID(uuid_struct)            COPY_UUID_128(uuid_struct,0xb2,0xb7,0x00,0x00,0x00,0x02,0x4c,0xb2,0xb3,0x4a,0x65,0x50,0xcc,0x0e,0x99,0x8c)
#define COPY_RELAY_SERVICE_UUID(uuid_struct)          COPY_UUID_128(uuid_struct,0xb2,0xb7,0x00,0x00,0x00,0x03,0x4c,0xb2,0xb3,0x4a,0x65,0x50,0xcc,0x0e,0x99,0x8c)
#define COPY_SERVO_SERVICE_UUID(uuid_struct)          COPY_UUID_128(uuid_struct,0xb2,0xb7,0x00,0x00,0x00,0x04,0x4c,0xb2,0xb3,0x4a,0x65,0x50,0xcc,0x0e,0x99,0x8c)

#define COPY_HUMIDITY_CHAR_UUID(uuid_struct)    	  COPY_UUID_128(uuid_struct,0x00,0x00,0x2a,0x6f,0x00,0x00,0x10,0x00,0x80,0x00,0x00,0x80,0x5f,0x9b,0x34,0xfb)
#define COPY_PRESSURE_CHAR_UUID(uuid_struct)    	  COPY_UUID_128(uuid_struct,0x00,0x00,0x2a,0x6d,0x00,0x00,0x10,0x00,0x80,0x00,0x00,0x80,0x5f,0x9b,0x34,0xfb)
#define COPY_TEMPERATURE_CHAR_UUID(uuid_struct)    	  COPY_UUID_128(uuid_struct,0x00,0x00,0x2a,0x6e,0x00,0x00,0x10,0x00,0x80,0x00,0x00,0x80,0x5f,0x9b,0x34,0xfb)
#define COPY_DIGITAL_CHAR_UUID(uuid_struct)    		  COPY_UUID_128(uuid_struct,0x00,0x00,0x2a,0x56,0x00,0x00,0x10,0x00,0x80,0x00,0x00,0x80,0x5f,0x9b,0x34,0xfb)
#define COPY_ANALOG_CHAR_UUID(uuid_struct)    		  COPY_UUID_128(uuid_struct,0x00,0x00,0x2a,0x58,0x00,0x00,0x10,0x00,0x80,0x00,0x00,0x80,0x5f,0x9b,0x34,0xfb)

/**
 * @brief  Event handler
 * @param  Event: Address of the buffer holding the Event
 * @retval Ack: Return whether the Event has been managed or not
 */
static SVCCTL_EvtAckStatus_t BLEDevice_Event_Handler(void *Event)
{
	SVCCTL_EvtAckStatus_t 					return_value;
	hci_event_pckt 							*event_pckt;
	evt_blue_aci 							*blue_evt;
	aci_gatt_attribute_modified_event_rp0	*attribute_modified;
	TEMPLATE_STM_App_Notification_evt_t 	Notification;

	return_value = SVCCTL_EvtNotAck;
	event_pckt = (hci_event_pckt *)(((hci_uart_pckt*)Event)->data);

	switch(event_pckt->evt)
	{
	case EVT_VENDOR:
	{
		blue_evt = (evt_blue_aci*)event_pckt->data;
		switch(blue_evt->ecode)
		{
		case EVT_BLUE_GATT_ATTRIBUTE_MODIFIED:
		{
			attribute_modified = (aci_gatt_attribute_modified_event_rp0*)blue_evt->data;
			if(attribute_modified->Attr_Handle == (aBLEDeviceContext.ServoSvcHdle + 2))
			{
				/**
				 * Descriptor handle
				 */
				return_value = SVCCTL_EvtAckFlowEnable;
				/**
				 * Notify to application
				 */
				uint16_t	value;
				value = (uint16_t)attribute_modified->Attr_Data[0] + 256 * (uint16_t)attribute_modified->Attr_Data[1];
				SetServoMotorPosition(value);
			}
		}
		break;

		default:
			break;
		}
	}
	break; /* HCI_EVT_VENDOR_SPECIFIC */

	default:
		break;
	}

	return(return_value);
}/* end SVCCTL_EvtAckStatus_t */


/* Public functions ----------------------------------------------------------*/

/**
 * @brief  Service initialization
 * @param  None
 * @retval None
 */
void SVCCTL_BLEDeviceInitCustomSvc(void)
{

	Char_UUID_t  uuid16;

	/**
	 *	Register the event handler to the BLE controller
	 */
	SVCCTL_RegisterSvcHandler(BLEDevice_Event_Handler);

	/**
	 *  Peer To Peer Service
	 *
	 * Max_Attribute_Records = 2*no_of_char + 1
	 * service_max_attribute_record = 1 for Template service +
	 *                                2 for Template Write characteristic +
	 *                                2 for Template Notify characteristic +
	 *                                1 for client char configuration descriptor +
	 *
	 */

	// THP Sensing service
	COPY_THPSENSOR_SERVICE_UUID(uuid16.Char_UUID_128);
	aci_gatt_add_service(UUID_TYPE_128,
			(Service_UUID_t *) &uuid16,
			PRIMARY_SERVICE,
			10, /*Max_Attribute_Records*/
			&(aBLEDeviceContext.THPSensorSvcHdle));

	//  Add Characteristics
	COPY_HUMIDITY_CHAR_UUID(uuid16.Char_UUID_128);
	aci_gatt_add_char(aBLEDeviceContext.THPSensorSvcHdle,
			UUID_TYPE_128, &uuid16,
			2,
			CHAR_PROP_READ | CHAR_PROP_NOTIFY,
			ATTR_PERMISSION_NONE,
			GATT_NOTIFY_ATTRIBUTE_WRITE, /* gattEvtMask */
			10, /* encryKeySize */
			1, /* isVariable */
			&(aBLEDeviceContext.HumidityCharHdle));
	COPY_PRESSURE_CHAR_UUID(uuid16.Char_UUID_128);
	aci_gatt_add_char(aBLEDeviceContext.THPSensorSvcHdle,
			UUID_TYPE_128, &uuid16,
			4,
			CHAR_PROP_READ | CHAR_PROP_NOTIFY,
			ATTR_PERMISSION_NONE,
			GATT_NOTIFY_ATTRIBUTE_WRITE, /* gattEvtMask */
			10, /* encryKeySize */
			1, /* isVariable */
			&(aBLEDeviceContext.PressureCharHdle));
	COPY_TEMPERATURE_CHAR_UUID(uuid16.Char_UUID_128);
	aci_gatt_add_char(aBLEDeviceContext.THPSensorSvcHdle,
			UUID_TYPE_128, &uuid16,
			2,
			CHAR_PROP_READ | CHAR_PROP_NOTIFY,
			ATTR_PERMISSION_NONE,
			GATT_NOTIFY_ATTRIBUTE_WRITE, /* gattEvtMask */
			10, /* encryKeySize */
			1, /* isVariable */
			&(aBLEDeviceContext.TemperatureCharHdle));

	// LED service
	COPY_LED_SERVICE_UUID(uuid16.Char_UUID_128);
	aci_gatt_add_service(UUID_TYPE_128,
			(Service_UUID_t *) &uuid16,
			PRIMARY_SERVICE,
			10, /*Max_Attribute_Records*/
			&(aBLEDeviceContext.LedSvcHdle));

	//  Add Characteristics
	COPY_DIGITAL_CHAR_UUID(uuid16.Char_UUID_128);
	aci_gatt_add_char(aBLEDeviceContext.LedSvcHdle,
			UUID_TYPE_128, &uuid16,
			4,
			CHAR_PROP_READ | CHAR_PROP_WRITE,
			ATTR_PERMISSION_NONE,
			GATT_NOTIFY_ATTRIBUTE_WRITE, /* gattEvtMask */
			10, /* encryKeySize */
			1, /* isVariable */
			&(aBLEDeviceContext.LedDigitalCharHdle));

	// Relays service
	COPY_RELAY_SERVICE_UUID(uuid16.Char_UUID_128);
	aci_gatt_add_service(UUID_TYPE_128,
			(Service_UUID_t *) &uuid16,
			PRIMARY_SERVICE,
			10, /*Max_Attribute_Records*/
			&(aBLEDeviceContext.RelaySvcHdle));

	//  Add Characteristics
	COPY_DIGITAL_CHAR_UUID(uuid16.Char_UUID_128);
	aci_gatt_add_char(aBLEDeviceContext.RelaySvcHdle,
			UUID_TYPE_128, &uuid16,
			2,
			CHAR_PROP_READ | CHAR_PROP_WRITE,
			ATTR_PERMISSION_NONE,
			GATT_NOTIFY_ATTRIBUTE_WRITE, /* gattEvtMask */
			10, /* encryKeySize */
			1, /* isVariable */
			&(aBLEDeviceContext.RelayDigitalCharHdle));

	// ServoMotor service
	COPY_SERVO_SERVICE_UUID(uuid16.Char_UUID_128);
	aci_gatt_add_service(UUID_TYPE_128,
			(Service_UUID_t *) &uuid16,
			PRIMARY_SERVICE,
			10, /*Max_Attribute_Records*/
			&(aBLEDeviceContext.ServoSvcHdle));

	//  Add Characteristics
	COPY_ANALOG_CHAR_UUID(uuid16.Char_UUID_128);
	aci_gatt_add_char(aBLEDeviceContext.ServoSvcHdle,
			UUID_TYPE_128, &uuid16,
			2,
			CHAR_PROP_READ | CHAR_PROP_WRITE,
			ATTR_PERMISSION_NONE,
			GATT_NOTIFY_ATTRIBUTE_WRITE, /* gattEvtMask */
			10, /* encryKeySize */
			1, /* isVariable */
			&(aBLEDeviceContext.ServoAnalogCharHdle));
	return;
}

/**
 * @brief  Characteristic update
 * @param  UUID: UUID of the characteristic
 * @param  Service_Instance: Instance of the service to which the characteristic belongs
 *
 */
tBleStatus BLEDevice_STM_App_Update_Char(uint16_t UUID, uint8_t *pPayload)
{
	tBleStatus result = BLE_STATUS_INVALID_PARAMS;
	switch(UUID)
	{
	case 0x0000:
		result = aci_gatt_update_char_value(aBLEDeviceContext.THPSensorSvcHdle,
				aBLEDeviceContext.HumidityCharHdle,
				0, /* charValOffset */
				2, /* charValueLen */
				(uint8_t *)  pPayload);
		break;

	case 0x0001:
		result = aci_gatt_update_char_value(aBLEDeviceContext.THPSensorSvcHdle,
				aBLEDeviceContext.PressureCharHdle,
				0, /* charValOffset */
				4, /* charValueLen */
				(uint8_t *)  pPayload);
		break;

	case 0x0002:
		result = aci_gatt_update_char_value(aBLEDeviceContext.THPSensorSvcHdle,
				aBLEDeviceContext.TemperatureCharHdle,
				0, /* charValOffset */
				2, /* charValueLen */
				(uint8_t *)  pPayload);
		break;

	default:
		break;
	}

	return result;
}/* end TEMPLATE_STM_Init() */




