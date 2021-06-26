/*
  Test
*/
#ifndef ZEROHEX_H
#define ZEROHEX_H

#include "Arduino.h"

class vector {

	public:
		float x;
		float y;
		float z;
		
		float xs;
		float ys;
		float zs;
		
		float xt;
		float yt;
		float zt;
		int oldmillis;
		
		vector();
		
		void reset();
		
		void setto(float nx, float ny, float nz);
		
		void setto(vector v);
		
		bool equalegS(vector v);
		
		float move2D(float pps);
};

class gaitEngine {
	
	private:
		float gaitStepN = 0;
		
	public:
		byte gSeq[120][6];
		byte legMoving[6];
		float gaitSpeed = 1.0; // control gait speed by adjusting gaitStepN progression
		float legSpeed = 1.0; // control gait speed by adjusting leg lift function
		float stepHeight = 20;
		float targetDeadZone = 3; // allow off center leg position
		float walkX = 0; // inputs for walking in X,Y and rotation
		float walkY = 0;
		float walkR = 0;
		float traX, traY, traZ; // current body translation
		float rotX, rotY, rotZ; // current for body rotation
		// hexapod constants
		int coxa  = 11.6;
		int femur = 44;
		int tibia = 69;
		
		vector coxaPos[6]; // body position
		vector leg[6]; //current leg positions (without body translation/rotation)
		vector legS[6]; //default leg positions
		vector legT[6]; //target leg positions
		vector legIK[6]; //leg positions for body translation/rotation
		
		float legAngle[6][3];
		
		gaitEngine();
		
		void gaitStep();
		
		// Body Translation/Rotation
		void runBodyIK();
		
		int runLegIK();
		
		float fsin(float x);
		float fcos(float x);
		
};

#endif
