#ifndef LIMB_H
#define LIMB_H

#include "Utils.h"
#include "Control.h" //this should be removed and lamp class should be it's own file

#define NO_BOUNDS 361.0

#define BASE_INCREMENT 15.0
#define FINE_INCREMENT 5.0
#define SUPER_FINE_INCREMENT 3.0

#define MAX_IK_CYCLES 1000

//	Denavit-Hartenberg
//		a          - distance from joint (i-1) to (i) on the Xi axis			(LINK LENGTH) fixed
//		al (alpha) - angle between Zi-1 axis and Zi axis about the Xi axis		(LINK TWIST)  fixed
//		d		   - distance from joint (i-1) to (i) on the Zi-1 axis			(LINK OFFSET) prismatic variable (fixed for revolute)
//		th (theta) - angle between Xi-1 axis and Xi axis about the Zi-1 axis	(JOINT ANGLE) revolute variable  (fixed for prismatic)

//		d or th is the "joint variable" (q) eg. driving parameter (actuator)

//		__________________________________________________________________
//		|-------------Rules For Defining Reference Frames:---------------|
//		|																 |
//		|	Zi-1 axis must be the axis of actuation for joint i			 |
//		|																 |
//		|	Axis Xi must be perpendicular to and intersect the Zi-1 axis |
//		|																 |
//		|	Axis Yi is derived from Zi and Xi per RHR					 |
//		|________________________________________________________________|
//		
//

struct pos
{
	float mX;
	float mY;
	float mZ;
};

float CalcDist(pos p1, pos p2);

class link
{
public:
	link(float a, float al, float d, float th, bool stablized = false, float jv_max = NO_BOUNDS, float jv_min = NO_BOUNDS);

	link* SetParent(link* parent)
	{
		return mP = parent;
	}

	link* SetChild(link* child)
	{
		return mC = child;
	}

	float SetJVRevo(float jv)
	{
		if (mStablized == true)
		{
			mC->SetJVRevo( (mC->GetTH()) - (jv-mTH) );
		}

		if ((jv > mJV_max) && (mJV_max != NO_BOUNDS)) 
		{
			//printf("JV Set Beyond max bound!\n");
			return mTH = mJV_max;
		}
		else if ((jv < mJV_min) && (mJV_min != NO_BOUNDS)) 
		{
			//printf("JV Set Beyond min bound!\n");
			return mTH = mJV_min; 
		}
		else 
		{
			return mTH = jv;
		}
	}

	float IncrementJVRevo(float inc)
	{
		return SetJVRevo(mTH + inc);
	}

	float SetJVPris(float jv)
	{
		return mD = jv;
		//return mDH.at(0).at(2) = jv;
	}
	
	void CalcHT();
	
	std::vector<std::vector<float>> mHT;
	
	float GetA() { return mA; }
	float GetAL() { return mAL; }
	float GetD() { return mD; }
	float GetTH() { return mTH; }

	float GetJV_max() { return mJV_max; }
	float GetJV_min() { return mJV_min; }

protected:

	float mA;
	float mAL;
	float mD;
	float mTH;

	float mJV_max;
	float mJV_min;

	link* mP;
	link* mC;
	bool mStablized; //This means that the next link's orentation is independent of this link (eg. this link is a parallel 4-bar)

};

class arm
{
public:

	arm() {};

	pos CalcJointPosition(int j, int frame);

	std::vector<link*>* GetLinks()
	{
		return &mLinks;
	}

	link* AddLink(link* l) //in the future modify to support branching arms
	{
		if (mLinks.empty() == false)//for any link other than the first
		{
			mLinks.back()->SetChild(l);
			l->SetParent(mLinks.back());
		};
		
		mLinks.push_back(l);
		return l;
	}

protected:

	std::vector<link*> mLinks;

};

class lamp //control.h should be removed and lamp class should be it's own file
{
public:

	lamp(Conductor* dux);

	pos SetPosition(float L1_rev, float L2_rev);

	float GetJointAngle(int link);

	float SetJointAngle(int link, float val);

	float IncJointAngle(int link, float val);

	pos CalcJointPosition(int j, int frame);

	float JVtest(pos goal, int j, float inc);

	float FindPlane(pos goal);

	bool CalcIK(pos goal);

	int DHtoServo(float val);
	
	void PrettyWrist();

	void Actuate();

	void IKto(int x, int y, int z)
	{
		pos ikpos = { float(x), float(y), float(z) };
		CalcIK(ikpos);
		Actuate();
	}

protected:

	arm mArm;

	Conductor* mP_dux;

	servo* mWST0;
	servo* mWST1;
	servo* mARM3;
	servo* mARM1;
	servo* mARM0;
};
#endif // !LIMB_H
