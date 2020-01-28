#include <stdint.h>
#include "i2c_bus.h"
/*#include "boards.h"
#include "nrf_log.h"
#include "nrf_drv_twi.h"
#include "app_util_platform.h"
#include "nordic_common.h"*/


/* TWI instance ID. */
#define TWI_INSTANCE_ID     0

/* Buffer for samples read from temperature sensor. */
//static uint8_t m_sample;
/* Indicates if operation on TWI has ended. */
//static volatile bool m_xfer_done = false;
/* TWI instance. */
//static const nrf_drv_twi_t m_twi = NRF_DRV_TWI_INSTANCE(TWI_INSTANCE_ID);

/**
 * @brief TWI events handler.
 */
/*void twi_handler(nrf_drv_twi_evt_t const * p_event, void * p_context)
{
    switch (p_event->type)
    {
        case NRF_DRV_TWI_EVT_DONE:
            if (p_event->xfer_desc.type == NRF_DRV_TWI_XFER_RX)
            {
                //data_handler(m_sample);
            }
            //m_xfer_done = true;
            i2c_bus::GetInstance().complete_transfer();
            break;
        default:
            break;
    }
}*/

i2c_bus::i2c_bus(void)
{
    /*ret_code_t  err_code;
    const nrf_drv_twi_config_t twi_bus_config = {
       .scl                = ARDUINO_SCL_PIN,
       .sda                = ARDUINO_SDA_PIN,
       .frequency          = NRF_TWI_FREQ_100K,
       .interrupt_priority = APP_IRQ_PRIORITY_HIGH,
       .clear_bus_init     = false
    };

    NRF_LOG_INFO("Called i2c_bus ctor.");
    err_code = nrf_drv_twi_init(&m_twi, &twi_bus_config, twi_handler, NULL);
    APP_ERROR_CHECK(err_code);

    m_xfer_done = false;
    nrf_drv_twi_enable(&m_twi);*/
}

i2c_bus::~i2c_bus(void)
{
    //printf("Called i2c_bus destructor.");
}

i2c_bus& i2c_bus::GetInstance(void)
{
    static i2c_bus  instance;
    return instance;
}

void i2c_bus::setHandle(I2C_HandleTypeDef *ph)
{
	pHandle = ph;
}

void i2c_bus::init(uint8_t dev_addr)
{
    mDevAddr = dev_addr << 1;
}

HAL_StatusTypeDef i2c_bus::write(uint8_t *buffer, uint8_t len)
{
	HAL_StatusTypeDef      err_code;
	mXferDone = false;
    err_code = HAL_I2C_Master_Transmit_IT(pHandle, mDevAddr, buffer, len);
    return err_code;
}

HAL_StatusTypeDef i2c_bus::read(uint8_t *buffer, uint8_t len)
{
	HAL_StatusTypeDef      err_code;
	mXferDone = false;
    err_code = HAL_I2C_Master_Receive_IT(pHandle, mDevAddr, buffer, len);
    return err_code;
}

void HAL_I2C_MasterTxCpltCallback(I2C_HandleTypeDef *hi2c)
{
	i2c_bus::GetInstance().complete_transfer();
}

void HAL_I2C_MasterRxCpltCallback(I2C_HandleTypeDef *hi2c)
{
	i2c_bus::GetInstance().complete_transfer();
}

void HAL_I2C_ErrorCallback(I2C_HandleTypeDef *hi2c)
{
	i2c_bus::GetInstance().setError(hi2c->ErrorCode);
	i2c_bus::GetInstance().complete_transfer();
}
