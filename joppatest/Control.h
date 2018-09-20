#ifndef CONTROL_H
#define CONTROL_H

#include <softPwm.h>
#include <pthread.h>
#include <errno.h>

#include "wiringPi.h"
#include "wiringPiI2C.h"
#include "Utils.h"

#define DEVICE_ID 0x40

//PCA9685 hexidecimal register addresses.
#define MODE1 0x00
#define MODE2 0x01

#define LED0_ON_L 0x06
#define LED0_ON_H 0x07
#define LED0_OFF_L 0x08
#define LED0_OFF_H 0x09

#define ALL_LED_ON_L 0xFA
#define ALL_LED_ON_H 0xFB
#define ALL_LED_OFF_L 0xFC
#define ALL_LED_OFF_H 0xFD

#define PRESCALE 0xFE
//---------------------------------------

#define RESTART  0x80
#define SLEEP  0x10
#define ALLCALL  0x01
#define INVRT  0x10
#define OUTDRV  0x04

//Slow Mode Tuners
#define SLOW_INC 30
#define SLOW_DELAY 100

class Conductor
{
public:
	Conductor()
	{
		//wiringPiSetup(); //Initialize WiringPI
		int filehandle = wiringPiI2CSetup(DEVICE_ID);
		if (filehandle == -1)
		{
			printf("error in i2c setup for device ID %i : %s\n", DEVICE_ID, strerror(errno));
		}
		else
		{
			printf("i2c setup successful!\n");
			mFD = filehandle;
		}

		set_all_pwm(0, 0);
		wiringPiI2CWriteReg8(mFD, MODE2, OUTDRV);
		wiringPiI2CWriteReg8(mFD, MODE1, ALLCALL);
		delay(5);//  wait for oscillator
		int mode1 = wiringPiI2CReadReg8(mFD, MODE1);
		mode1 = (mode1 & ~SLEEP);// wake up(reset sleep)
		wiringPiI2CWriteReg8(mFD, MODE1, mode1);
		delay(5);//  wait for oscillator

		set_pwm_freq(60); //60 Hz for servos

	}

	int set_pwm_freq(int freq_hz)
	{
		int psval = 25000000.0;  // 25MHz
		psval /= 4096.0;  // 12 - bit
		psval /= float(freq_hz);
		psval -= 1.0;

		printf("Setting PWM frequency to %i Hz\n", freq_hz);
		printf("Estimated pre-scale: %i\n", psval);

		psval = int(floor(psval + 0.5));

		int oldmode = wiringPiI2CReadReg8(mFD, MODE1);
		int newmode = ((oldmode & 0x7F) | 0x10);    // sleep
		wiringPiI2CWriteReg8(mFD, MODE1, newmode); //set to sleep mode
		wiringPiI2CWriteReg8(mFD, PRESCALE, psval);
		wiringPiI2CWriteReg8(mFD, MODE1, oldmode); //set back to pervious mode
		delay(5);
		wiringPiI2CWriteReg8(mFD, MODE1, oldmode | 0x80);
		return 0;
	}


	int set_pwm(int channel, int on, int off);

	int set_all_pwm(int on, int off);

	void test()
	{
		printf("Testing servos on channels 0 & 1, for 10 cycles\n");

		for (int i = 0; i < 10; i++)
		{

			int servo_min = 150;//  Min pulse length out of 4096
			int servo_max = 600;//  Max pulse length out of 4096

			set_pwm(0, 0, servo_min);
			set_pwm(1, 0, servo_min);
			delay(1000);
			set_pwm(0, 0, servo_max);
			set_pwm(1, 0, servo_max);
			delay(1000);
		}
	}

protected:

	int mFD;
};


class servo
{
public:

	servo(Conductor* dux, int channel, int range = 180, int max = 180, int min = 0) : mDux(dux), mChannel(channel), mRange(range), mMax(max), mMin(min), mCurrentAngle(0) {}

	int ConvertAngle(int val)
	{
		if (val > mMax)
		{
			val = mMax;
			printf("Sevo set past coded max; setting val to %i\n", val);
		}
		else if (val < mMin)
		{
			val = mMin;
			printf("Sevo set past coded min; setting val to %i\n", val);
		}

		int ret = (int)(125.0 + ((float)val*(450.0/(float)mRange) )); //0:125, 180:575
		//printf("raw val: %i \n", ret);
		return ret;
	}

	int SetAngle(int val)
	{
		mCurrentAngle = ConvertAngle(val);
		return mDux->set_pwm(mChannel, 0, mCurrentAngle);
	}

	int SetAngleSlow(int val)
	{
		int ret = -1;
		int GoalAng = val;

		printf("Current Angle = %i\n", mCurrentAngle);
		printf("Goal = %i\n", GoalAng);

		while (mCurrentAngle < GoalAng)
		{
			if((mCurrentAngle + SLOW_INC) > GoalAng)
			{
				mCurrentAngle = GoalAng;
			}
			else
			{
				mCurrentAngle += SLOW_INC;
			}
			printf("Slow Servo Angle = %i\n", mCurrentAngle);

			ret = mDux->set_pwm(mChannel, 0, ConvertAngle(mCurrentAngle));
			delay(SLOW_DELAY);
		}

		while (mCurrentAngle > GoalAng)
		{
			if ((mCurrentAngle - SLOW_INC) < GoalAng)
			{
				mCurrentAngle = GoalAng;
			}
			else
			{
				mCurrentAngle -= SLOW_INC;
			}
			printf("Slow Servo Angle = %i\n", mCurrentAngle);

			ret = mDux->set_pwm(mChannel, 0, ConvertAngle(mCurrentAngle));
			delay(SLOW_DELAY);
		}

		return ret;
	}

protected:
	int mRange;

	int mMax;
	int mMin;

	int mChannel;
	Conductor* mDux;

	int mCurrentAngle;
};
#endif