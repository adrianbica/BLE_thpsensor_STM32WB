/*
 * ServoMotor.cpp
 *
 *  Created on: Jan 25, 2020
 *      Author: adrian
 */

#include "ServoMotor.h"

#include "stm32wbxx_hal.h"

extern TIM_HandleTypeDef htim16;

ServoMotor::ServoMotor(uint16_t minv, uint16_t maxv, uint16_t zerov, uint16_t minp, uint16_t maxp, uint16_t zerop)
{
	mMinLimitValue = minv;
	mMaxLimitValue = maxv;
	mZeroValue = zerov;
	mMinPulse = minp;
	mMaxPulse = maxp;
	mZeroPulse = zerop;
	SetCurrValue(zerop);
}

void ServoMotor::SetCurrValue(uint16_t val)
{
	uint32_t temp;

	mCurrValue = val;
	temp = (uint32_t)(mMaxPulse - mMinPulse) * (uint32_t)val;
	temp = temp / (mMaxLimitValue - mMinLimitValue) + mMinPulse;
	if(temp > mMaxPulse) {
		temp = mMaxPulse;
	}
	if(temp < mMinPulse) {
		temp = mMinPulse;
	}

	htim16.Instance->CCR1 = (uint16_t)temp;
}

uint16_t ServoMotor::GetCurrValue(void)
{
	return mCurrValue;
}





