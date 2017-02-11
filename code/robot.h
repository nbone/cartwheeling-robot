/* CS572 - Robotics
 * Final Project
 * 25 May 2010
 * Nicholas Bone
 */

#ifndef ROBOT_H
#define ROBOT_H

// Unfortunately the "extra" ifndefs below are necessary because the compiler is stupid
#ifndef ANN_H
#include "ann.h"
#endif

typedef struct {
	float a[ANN_NUM_INPUTS];
} InputArray;

// NOTE: made these global rather than in a struct because of RobotC runtime problems with nested structs:
float r_angle1;	// normalized angle of REAR (or ONLY) TOUCHING leg, range: [0,1]
float r_angle2;	// normalized angle of NEXT (or FRONT TOUCHING) leg, range: [0,1]
float r_angle3;	// normalized angle of TOP (in the air) leg, range: [0,1]
int r_power1; 		// current motor power to leg 1, range: [0,100]
int r_power2; 		// current motor power to leg 2, range: [0,100]
int r_power3; 		// current motor power to leg 3, range: [0,100]
//ANN r_ann1; 		// state-value function approximator for case where ONE toe is touching
//ANN r_ann2; 		// state-value function approximator for case where TWO toes are touching
LMS r_ann1; 		// state-value function approximator for case where ONE toe is touching
LMS r_ann2;			// state-value function approximator for case where TWO toes are touching
int r_toeState;	// bit mask combination of TOE1, TOE2, TOE3
bool r_isTwoToes;	// true = two toes are touching (or 3?); false = one toe is touching (or 0?)
bool r_isTrainingMode;	// true = learn with special training rewards, but just let legs float;
						// false = act under own power
bool r_isHardcodeMode;	// true = learn normally, but follow hardcoded behavior

typedef struct {
	int leg1min;	// minimum encoder value of leg1 motor
	int leg1max;	// maximum encoder value of leg1 motor
	int leg2min;	// minimum encoder value of leg2 motor
	int leg2max;	// maximum encoder value of leg2 motor
	int leg3min;	// minimum encoder value of leg3 motor
	int leg3max;	// maximum encoder value of leg3 motor
} RobotSettings;

float Gaussian(float x, float c);

float EstimateAngleChange(float angle, int power);

void R_Init();

void R_SetTrainingMode(bool bTrain);

void R_SetStateFromSensors(RobotSettings& rs);

void R_SetMotorsFromState();

void R_SetANNInputsFromLegAngle(int eLeg, float angle);

void R_SetANNInputsFromState();

void R_GetANNInputs(InputArray& inputs);

void R_SetANNInputs(InputArray& inputs);

float R_Activate();

int R_ChooseLegPower(int eLeg);

float R_GetRewardSignal(int prevToeState);

void R_Train(int prevToeState, float targetValue);

void R_SenseActLearn(RobotSettings& rs);

#endif
