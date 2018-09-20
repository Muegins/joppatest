#ifndef UTILS_H
#define UTILS_H

//#define RELEASE
#define DEBUG


#include <vector>
#include <time.h>
#include <stdio.h>
#include <string.h>
#include <assert.h>
#include <stdarg.h>
#include <stdlib.h>
#include <math.h>

//Terrain Defines
#define AIR			0x00
#define GROUND		0x01
#define PLATFORM	0x02
#define WATER		0x03
//

//Screen size defines //These must be a whole multiple of the camera size otherwise pixel stretching occurs
#define SCREEN_W 1344
#define SCREEN_H 756
//

//Direction Defines. Used by actor and terrain systems
#define DOWN  3
#define UP	  2
#define LEFT  1 // must be 1 because of SDL draw function
#define RIGHT 0 // must be 0 because of SDL draw function
//

//Time Defines
#define MS_PER_FRAME 16 //this is normal speed
//#define MS_PER_FRAME 40 //useful slow speed
#define FRAMES_PER_SECOND (1/MS_PER_FRAME)*1000 //62.5 @16ms

#define CLOCKS_PER_FRAME (CLOCKS_PER_SEC/FRAMES_PER_SECOND)
//

class Time
{
public:

	Time() : mCycle(0) {};

	int ProgressCycle()
	{
		mCycle = (mCycle + 1);
		return mCycle;
	}

	int GetCurrentCycle()
	{
		return mCycle;
	}

	int GetCurrentMS()
	{
		return (mCycle*MS_PER_FRAME);
	}
protected:

	int mCycle;
};

class Random
{
public:
	Random();

	int RandRange(int min, int max);
protected:

	int mSeed;
};

class Logger
{
public:
	Logger()
	{
		FILE* log = fopen("log.txt", "w");
	}
	
	void LogPrintf(char* fmt, ...);

	void PrintMatrixToLog(std::vector<std::vector<float>>* m);

protected:

	FILE* mLog;
};

int GetRealTimeMS();

float sq(float val);

float GetSign(float x);

//int CalcDistance(int x1, int y1, int x2, int y2);

int EvenOutcomes(int NumberofOutcomes);

bool IsEven(int i);

float DegsToRads(float deg);

float RadsToDegs(float rad);

float sind(double th);

float cosd(double th);

float tand(double th);

std::vector<std::vector<float>> MultipyMatrix(std::vector<std::vector<float>>* m1, std::vector<std::vector<float>>* m2);


extern Time* gTime;
extern Random* gRandom;
extern Logger* gLogger;

#endif
