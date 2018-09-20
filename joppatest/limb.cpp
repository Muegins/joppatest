#include "limb.h"
#include "Control.h"

float CalcDist(pos p1, pos p2)
{
	float deltaX = abs(p2.mX - p1.mX);
	float deltaY = abs(p2.mY - p1.mY);
	float deltaZ = abs(p2.mZ - p1.mZ);

	float dist = sqrt(sq(deltaX) + sq(deltaY) + sq(deltaZ));
	return dist;
}

//link
link::link(float a, float al, float d, float th, bool stablized, float jv_max, float jv_min) : mP(NULL), mC(NULL), mStablized(stablized), mJV_max(jv_max), mJV_min(jv_min)
{
//	mDH.resize(1);
//	for (int i = 0; i < 1; i++)
//	{
//		mDH.at(i).resize(4);
//	}
//	mDH.at(0).at(0) = a;
//	mDH.at(0).at(1) = al;
//	mDH.at(0).at(2) = d;
//	mDH.at(0).at(3) = th;

	mA = a;
	mAL = al;
	mD = d;
	mTH = th;
}

void link::CalcHT()
{

//	float a = mDH.at(0).at(0);
//	float al = mDH.at(0).at(1);
//	float d = mDH.at(0).at(2);
//	float th = mDH.at(0).at(3);

	float a = mA;
	float al = mAL;
	float d = mD;
	float th = mTH;

	mHT =
	{
		{ cosd(th) , -sind(th)*cosd(al)  ,  sind(th)*sind(al)	 , a*cosd(th) },
		{ sind(th) ,  cosd(th)*cosd(al)  , -cosd(th)*sind(al)    , a*sind(th) },
		{ 0        ,  sind(al)           ,  cosd(al)             , d		  },
		{ 0		   ,  0				     ,  0			         , 1		  }
	};

	//FILE* fklog = fopen("FK_log.txt", "w");

	//gLogger->PrintMatrixToLog(&mHT);
}
//

//arm
pos arm::CalcJointPosition(int j, int frame)
{
	std::vector<std::vector<float>> T =
	{
		{ 1,0,0,0 },
		{ 0,1,0,0 },
		{ 0,0,1,0 },
		{ 0,0,0,1 }
	};

	for (int i = (frame); (j >= i && i >= frame); i++)
	{
		mLinks.at(i)->CalcHT();
		T = MultipyMatrix(&T, &(mLinks.at(i)->mHT));
	};

	pos ee;
	ee.mX = T.at(0).at(3);
	ee.mY = T.at(1).at(3);
	ee.mZ = T.at(2).at(3);

	return ee;
}
//

//lamp

lamp::lamp(Conductor* dux) : mP_dux(dux)
{
//--------------------Test Arm----------------------
//	link* l2 = new link(10, 0.0, 0.0, 30.0, true);
//	mArm.AddLink(l2);
//	link* l3 = new link(10, 0.0, 0.0, -30.0);
//	mArm.AddLink(l3);
//	link* l4 = new link(10, 0.0, 0.0, 0.0, l3);
//	mArm.AddLink(l4);
//	link* l5 = new link(10, 0.0, 0.0, 0.0, l4);
//	mArm.AddLink(l5);
//--------------------------------------------------

//--------------------Lamp Arm----------------------
	link* l0 = new link(0.00, -90.0, 2.29, 0.00); //foot
	mArm.AddLink(l0);
	link* l1 = new link(8.55, 0.00, 0.00, -70.00, true, -60.0,-120.0); // //ogram 1
	mArm.AddLink(l1);
	link* l2 = new link(1.25, 0.00, 0.00, 13.49); // joining plates
	mArm.AddLink(l2);
	link* l3 = new link(8.55, 0.00, 0.00, 45.0, true, 70.0, 10.0); // //ogram 2
	mArm.AddLink(l3);
	link* l4 = new link(1.82, 0.0, 0.00, 32.53); //wrist plates 
	mArm.AddLink(l4);
//--------------------------------------------------

	mWST0 = new servo(mP_dux, 0, 180, 150, 30);
	mWST1 = new servo(mP_dux, 1, 135, 135, 0);
	mARM3 = new servo(mP_dux, 2, 60, 60, 0);
	mARM1 = new servo(mP_dux, 3, 60, 60, 0);
	mARM0 = new servo(mP_dux, 4, 180);
}

pos lamp::SetPosition(float L1_rev, float L2_rev)
{
	mArm.GetLinks()->at(0)->SetJVRevo(L1_rev);
	mArm.GetLinks()->at(1)->SetJVRevo(L2_rev);

	return mArm.CalcJointPosition(1, 0);
}

float lamp::GetJointAngle(int link)
{
	return mArm.GetLinks()->at(link)->GetTH();
}

float lamp::SetJointAngle(int link, float val)
{
	return mArm.GetLinks()->at(link)->SetJVRevo(val);
}

float lamp::IncJointAngle(int link, float val)
{
	return mArm.GetLinks()->at(link)->IncrementJVRevo(val);
}

pos lamp::CalcJointPosition(int j, int frame)
{
	return mArm.CalcJointPosition(j, frame);
}

float lamp::JVtest(pos goal, int j, float inc)
{
	float JV_i = mArm.GetLinks()->at(j)->GetTH();

//	if ((JV_i == mArm.GetLinks()->at(j)->GetJV_max()) | (JV_i == mArm.GetLinks()->at(j)->GetJV_max()))
//	{
//		return 0.0;
//	}

	//Test positive increment
	float o_dist = CalcDist(CalcJointPosition(4, 0), goal);
	mArm.GetLinks()->at(j)->IncrementJVRevo(inc);
	float n_dist = CalcDist(CalcJointPosition(4, 0), goal); 
	mArm.GetLinks()->at(j)->SetJVRevo(JV_i);	
	
	if (n_dist > o_dist) //If the change results in a negative effect, test the negative increment
	{
		inc = (inc*-1);
		mArm.GetLinks()->at(j)->IncrementJVRevo((inc));
		n_dist = CalcDist(CalcJointPosition(4, 0), goal);
		mArm.GetLinks()->at(j)->SetJVRevo(JV_i);
	}

	if (n_dist > o_dist) //If the change still results in a negative effect, don't move the joint.
	{
		//printf("JV Test finds no positive adjustment\n");
		return 0.0; // Don't move this joint
	}

	float scale_factor = inc*(o_dist - n_dist); //Calculate Scale Factor

	return scale_factor;
}

float lamp::FindPlane(pos goal)
{
	float gX = DegsToRads(goal.mX);
	float gY = DegsToRads(goal.mY);

	float angle = RadsToDegs(atan(gY / gX));

	//printf("Base Angle: %f\n", angle);

	return angle;
}


bool lamp::CalcIK(pos goal) //Add Dynamic increment scaling
{
	SetJointAngle(0, FindPlane(goal));
	
	float delta = CalcDist(CalcJointPosition(4, 0), goal);
	float delta_i = delta;
	int time_out = 0;

	float increment = BASE_INCREMENT;

	while ( delta > 0.25 )
	{
		time_out++;
		
		increment = (BASE_INCREMENT*(delta / delta_i));

//		if (delta < 1)//Activate Super Fine Adjust Mode
//		{
//			increment = SUPER_FINE_INCREMENT;
//		} else
//		if (delta < 2)//Activate Fine Adjust Mode
//		{
//			increment = FINE_INCREMENT;
//		}
		
//		float sf_0 = (JVtest(goal, 0, increment));// *increment);
		float sf_1 = (JVtest(goal, 1, increment));// *increment);
		float sf_3 = (JVtest(goal, 3, increment));// *increment);
		//0, 1, 3 are the powered joints

//		float th_0 = IncJointAngle(0, sf_0);
		float th_1 = IncJointAngle(1, sf_1);
		float th_3 = IncJointAngle(3, sf_3);

		if (delta == CalcDist(CalcJointPosition(4, 0), goal))
		{
			printf("\nIK Convergence Failure\n");

			printf("\nCycle: %i\n", time_out);
			printf("Distance from target: %f\n", delta);
//			printf("JV_0 = %f\n", mArm.GetLinks()->at(0)->GetTH());
			printf("JV_1 = %f\n", mArm.GetLinks()->at(1)->GetTH());
			printf("JV_3 = %f\n", mArm.GetLinks()->at(3)->GetTH());

			return false;
		}

		delta = CalcDist(CalcJointPosition(4, 0), goal);

		//printf("\nCycle: %i\n", time_out);
		//printf("Distance from target: %f\n", delta);
		//printf("JV_0 = %f | SF_0 = %f\n", th_0, sf_0);
		//printf("JV_1 = %f | SF_1 = %f\n", th_1, sf_1);
		//printf("JV_3 = %f | SF_3 = %f\n", th_3, sf_3);

		//printf("\n%f,", th_3);
		//printf(" %f,", th_1);


		if (time_out == MAX_IK_CYCLES)
		{
			printf("\nIK reached max cycles\n");

			printf("\nCycle: %i\n", time_out);
			printf("Distance from target: %f\n", delta);
//			printf("JV_0 = %f\n", mArm.GetLinks()->at(0)->GetTH());
			printf("JV_1 = %f\n", mArm.GetLinks()->at(1)->GetTH());
			printf("JV_3 = %f\n", mArm.GetLinks()->at(3)->GetTH());

			return false;
		}
	}

	printf("\nCycle: %i\n", time_out);
	printf("Distance from target: %f\n", delta);
	//printf("JV_0 = %f\n", mArm.GetLinks()->at(0)->GetTH());
	printf("JV_1 = %f\n", mArm.GetLinks()->at(1)->GetTH());
	printf("JV_3 = %f\n", mArm.GetLinks()->at(3)->GetTH());

	return true;
};

int DHtoServo(float val)
{
	return 0;
}

void lamp::PrettyWrist()
{
	mWST0->SetAngle(100);
	mWST1->SetAngle(135);
	delay(100);
	mP_dux->set_all_pwm(0, 0);
}

void lamp::Actuate()
{
	int arm0 = (mArm.GetLinks()->at(0)->GetTH() + 90);
	mARM0->SetAngle(arm0);

	mWST0->SetAngle(-(mArm.GetLinks()->at(0)->GetTH()) +100);

	int arm1 = (mArm.GetLinks()->at(1)->GetTH() + 120);
	//printf("Arm1 DH = %f\n", mArm.GetLinks()->at(1)->GetTH());
	printf("Setting arm1 to %i\n", arm1);
	mARM1->SetAngle(arm1);

	int arm3 = (-1*(mArm.GetLinks()->at(3)->GetTH()) + 70);
	//printf("Arm3 DH = %f\n", mArm.GetLinks()->at(3)->GetTH());
	printf("Setting arm3 to %i\n", arm3);
	mARM3->SetAngle(arm3);

	delay(1000);
}

//
