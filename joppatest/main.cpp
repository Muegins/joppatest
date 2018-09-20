#include <cstdio>
#include <iostream>
#include "Utils.h"
#include "limb.h"
#include "Control.h"

using namespace std;

int main()
{

#define KINEMATICS_DEMO
//#define KINEMATICS_TEST
//#define CONTROL_TEST

	//gLogger = new Logger();

	Conductor dux = Conductor();
	lamp L = lamp(&dux);

#ifdef KINEMATICS_DEMO

	L.Actuate();
	L.PrettyWrist();
	
	int in;
	int running = true;
	while (running == true)
	{
		cin >> in;
		switch (in)
		{
		case -1: //terminate program
			running = false;
			break;
		case 0: //Stop servos
			dux.set_all_pwm(0, 0);
			break;
		case 1: //Demo Diamond
			L.IKto(8, 0, 18);
			L.IKto(8, -4, 15);
			L.IKto(8, 0, 12);
			L.IKto(8, 4, 15);
			L.IKto(8, 0, 18);
			break;

		case 2: //Demo square
			L.IKto(8, -4, 18);
			L.IKto(8, -4, 12);
			L.IKto(8, 4, 12);
			L.IKto(8, 4, 18);
			L.IKto(8, -4, 18);
			break;
		default:
			break;
		}
	}

	printf("Program Complete!\n");

#endif // KINEMATICS_DEMO

#ifdef KINEMATICS_TEST



	//L.SetJointAngle(0, 45);


	//In General:
	// (6.7 < sqrt(X^2+Y^2) < 15.1)
	// (8.1 < Z < 17.3)

	pos ikpos = { 8.0, 0.0, 10.0 }; 
	L.CalcIK(ikpos);

	pos pos0 = L.CalcJointPosition(0, 0);
	pos pos1 = L.CalcJointPosition(1, 0);
	pos pos2 = L.CalcJointPosition(2, 0);
	pos pos3 = L.CalcJointPosition(3, 0);
	pos pos4 = L.CalcJointPosition(4, 0);
	float d = CalcDist(pos0, pos4);

//#define PLOT_MODE
#ifdef PLOT_MODE

	FILE* out = fopen("plot.txt", "w");

	for (int plot_jv3 = 15; plot_jv3 <= 95; plot_jv3++) 
	{
		fprintf(out, "%i,", (int)plot_jv3);// JV3
	}

	for (int plot_jv1 = -130.0; plot_jv1 <= -50; plot_jv1++)
	{
		L.SetJointAngle(1, plot_jv1);
		fprintf(out,"\n%i,", (int)(L.GetJointAngle(1)));// JV1
		for (int plot_jv3 = 15; plot_jv3 <= 95; plot_jv3++)
		{
			L.SetJointAngle(3, plot_jv3);
			pos plot = L.CalcJointPosition(4, 0);
			float delta = CalcDist(plot, pos4);

			//fprintf(out," %i,", (int)(L.GetJointAngle(3)) );// JV3
			//printf(" %f,", plot.mX);// X
			//printf(" %f,", plot.mY);// Y
			fprintf(out," %f,", delta); // dist
		}
	}

	fclose(out);


#endif // PLOT

//#define TISMMODE
#ifdef TISMMODE

	printf("\nJ0 X: %f\n", pos0.mX);
	printf("J0 Y: %f\n", pos0.mY);
	printf("J0 Z: %f\n", pos0.mZ);

	printf("\nJ1 X: %f\n", pos1.mX);
	printf("J1 Y: %f\n", pos1.mY);
	printf("J1 Z: %f\n", pos1.mZ);

	printf("\nJ2 X: %f\n", pos2.mX);
	printf("J2 Y: %f\n", pos2.mY);
	printf("J2 Z: %f\n", pos2.mZ);

	printf("\nJ3: %f\n", pos3.mX);
	printf("J3: %f\n", pos3.mY);
	printf("J3: %f\n", pos3.mZ);

#endif

	printf("\nJ4 (End Effector) X: %f\n", pos4.mX);
	printf("J4 (End Effector) Y: %f\n", pos4.mY);
	printf("J4 (End Effector) Z: %f\n", pos4.mZ);
	//printf("dist from origin: %f\n", d);

	printf("\n");

#endif // KINEMATICS_TEST

#ifdef CONTROL_TEST



	//dux.test();

	servo wst0 = servo(&dux, 0, 180, 150, 30);
	servo wst1 = servo(&dux, 1, 135, 135, 0);
	servo arm3 = servo(&dux, 2, 60, 60, 0);
	servo arm1 = servo(&dux, 3, 60, 60, 0);
	servo arm0 = servo(&dux, 4, 180);

	int in;
	int running = true;
	while (running == true)
	{	
		cin >> in;
		if (in == -1)
		{
			running = false;
		}
		else
		{
			//dux.set_pwm(0, 0, in);
			arm1.SetAngle(in);
		}
	}

	printf("Program Complete!\n");

#endif // CONTROL_TEST
	
	dux.set_all_pwm(0, 0);

    return 0;
}