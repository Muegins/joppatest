#include "Utils.h"

Time* gTime = NULL;
Random* gRandom = NULL;
Logger* gLogger = NULL;

//int GetTimeF() //measures in frames
//{
//	int time = (clock() / CLOCKS_PER_FRAME);
//	return time;
//}

int GetRealTimeMS() //measures in milliseconds
{
	int time = (clock() / (CLOCKS_PER_SEC / 1000));
	return time;
}

float sq(float val)
{
	return (val*val);
}

float GetSign(float x)
{
	if (x > 0) { return  1; }
	if (x < 0) { return -1; }
	return 0;
}


int CalcDistance(int x1, int y1, int x2, int y2)
{
	int Xdelt = abs(x1 - x2);
	int Ydelt = abs(y1 - y2);

	return (int)sqrt((Xdelt*Xdelt) + (Ydelt*Ydelt));
}

int EvenOutcomes(int NumberofOutcomes)
{
	int	i = (rand() % (NumberofOutcomes)) + 1;
	//gCons->ConsPrintf("Roll : %i\n", i);
	return i;
}

Random::Random()
{
	mSeed = (int)time(NULL);
	srand(mSeed);
}

int Random::RandRange(int v1, int v2)
{
	int i;
	if (v1 < v2) //v1 is min
	{
		i = (v1 + (rand() % (1 + v2 - v1)));
		assert(i <= v2);
		assert(i >= v1);
	}
	else if (v2 < v1) //v2 is min
	{
		i = (v2 + (rand() % (1 + v1 - v2)));
		assert(i <= v1);
		assert(i >= v2);
	}
	else// v1 == v2
	{
		i = v1;
	}
	//gCons->ConsPrintf("Range Roll : %i\n", i);
	return i;
}

void Logger::PrintMatrixToLog(std::vector<std::vector<float>>* m)
{
	for (int i = 0; i < m->size(); i++) // for each row
	{
		fprintf(mLog, "{ ");
		for (int j = 0; j < m->at(i).size(); j++)// for each entry in that row
		{
			fprintf(mLog, "%f, ", m->at(i).at(j));
		}
		fprintf(mLog, "}\n");
	}
}

bool IsEven(int i)
{
	if ( (i % 2) == 0)
	{
		return true;
	}
	else
	{
		return false;
	}
}

float DegsToRads(float deg)
{
	return (deg / 180)*3.14;
}

float RadsToDegs(float rad)
{
	return (rad / 3.14)*180;
}

float sind(double th) //sin for degrees
{
	return (float)sin(DegsToRads(th));
}

float cosd(double th)//cos for degrees
{
	return (float)cos(DegsToRads(th));
}

float tand(double th)//tan for degrees
{
	return (float)tan(DegsToRads(th));
}

std::vector<std::vector<float>> MultipyMatrix(std::vector<std::vector<float>>* m1, std::vector<std::vector<float>>* m2)
{
	int common = (m1->at(0).size());
	if (common != (m2->size())) //Invalid Matrix Dimensions for multiplication
	{
		assert(false);
	}

	std::vector<std::vector<float>> out((m1->size()), std::vector<float>(m2->at(0).size())); //allocate product matrix


	int m1_height = m1->size();
	for (int i = 0; i < m1_height; i++) //for each row in m1
	{
		int m2_width = m2->at(0).size();
		for (int j = 0; j < m2_width; j++) //for each column in m2
		{
			out.at(i).at(j) = 0;
			for (int k = 0; k < common; k++)
			{
				out.at(i).at(j) += m1->at(i).at(k) * m2->at(k).at(j);
			}
		}
	}

	return out;
}
