/*
 * ServoMotor.h
 *
 *  Created on: Jan 25, 2020
 *      Author: adrian
 */

#ifndef INC_SERVOMOTOR_H_
#define INC_SERVOMOTOR_H_

#include <stdint.h>

using namespace std;

class ServoMotor
{
private:
	uint16_t	mMinLimitValue;
	uint16_t	mMaxLimitValue;
	uint16_t	mZeroValue;
	uint16_t	mMinPulse;
	uint16_t	mMaxPulse;
	uint16_t	mZeroPulse;
	uint16_t	mCurrValue;
public:
	ServoMotor(uint16_t minv, uint16_t maxv, uint16_t zerov, uint16_t minp, uint16_t maxp, uint16_t zerop);
	void SetCurrValue(uint16_t val);
	uint16_t GetCurrValue(void);
};


#endif /* INC_SERVOMOTOR_H_ */
