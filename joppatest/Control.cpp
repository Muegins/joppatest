#include "Control.h"


int Conductor::set_pwm(int channel, int on, int off)
{
	wiringPiI2CWriteReg8(mFD, (LED0_ON_L + channel*4), on & 0xFF);
	wiringPiI2CWriteReg8(mFD, (LED0_ON_H + channel * 4), on >> 8);
	wiringPiI2CWriteReg8(mFD, (LED0_OFF_L + channel * 4), off & 0xFF);
	wiringPiI2CWriteReg8(mFD, (LED0_OFF_H + channel * 4), off >> 8);
	return 0;
}

int Conductor::set_all_pwm(int on, int off)
{
	wiringPiI2CWriteReg8(mFD, ALL_LED_ON_L, on & 0xFF);
	wiringPiI2CWriteReg8(mFD, ALL_LED_ON_H, on >> 8);
	wiringPiI2CWriteReg8(mFD, ALL_LED_OFF_L, off & 0xFF);
	wiringPiI2CWriteReg8(mFD, ALL_LED_OFF_H, off >> 8);
	return 0;
}


