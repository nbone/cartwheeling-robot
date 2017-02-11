/* CS572 - Robotics
 * Final Project
 * 25 May 2010
 * Nicholas Bone
 */

#ifndef ROBOT_C
#define ROBOT_C

// Unfortunately the "extra" ifndefs below are necessary because the compiler is stupid
#ifndef CONSTANTS_H
#include "constants.h"
#endif
#ifndef ANN_C
#include "ann.c"	// no linker, so must include .c instead of .h
#endif
#ifndef ROBOT_H
#include "robot.h"
#endif

float Gaussian(float x, float c) {
	return exp(-GAUSSIAN_BETA * abs(x - c));
}

float EstimateAngleChange(float angle, int power) {
	const int POWER_DIVISOR = 200;	// TODO: determine a good value for this
	float newAngle = angle + ((float)power / POWER_DIVISOR);
	newAngle = max(0.0, min(1.0, newAngle));
	return newAngle;
}

void R_Init() {
	Init(r_ann1);
	Init(r_ann2);
	r_angle1 = -1.0;
	r_angle2 = -1.0;
	r_angle3 = -1.0;
	r_power1 = 0;
	r_power2 = 0;
	r_power3 = 0;
	r_toeState = 0;
	r_isTwoToes = false;
	r_isTrainingMode = false;
	r_isHardcodeMode = false;
}

void R_SetTrainingMode(bool bTrain) {
	r_isTrainingMode = bTrain;
	r_isHardcodeMode = false;
}

void R_SetHardcodeMode(bool bHC) {
	r_isHardcodeMode = bHC;
	r_isTrainingMode = false;
}

void R_SetStateFromSensors(RobotSettings& rs) {
	float a1 = (nMotorEncoder[leg1] - rs.leg1min) / ((float) rs.leg1max - rs.leg1min);
	float a2 = (nMotorEncoder[leg2] - rs.leg2min) / ((float) rs.leg2max - rs.leg2min);
	float a3 = (nMotorEncoder[leg3] - rs.leg3min) / ((float) rs.leg3max - rs.leg3min);
	r_toeState = 0;
	r_isTwoToes = false;
	if (SensorValue[toe1]) r_toeState |= TOE1;
	if (SensorValue[toe2]) r_toeState |= TOE2;
	if (SensorValue[toe3]) r_toeState |= TOE3;
	switch (r_toeState) {
		case TOE1 | TOE2:
			r_isTwoToes = true;
			/* fall through to next case */
		case TOE1:
			r_angle1 = a1;
			r_angle2 = a2;
			r_angle3 = a3;
			break;
		case TOE2 | TOE3:
			r_isTwoToes = true;
			/* fall through to next case */
		case TOE2:
			r_angle1 = a2;
			r_angle2 = a3;
			r_angle3 = a1;
			break;
		case TOE3 | TOE1:
			r_isTwoToes = true;
			/* fall through to next case */
		case TOE3:
			r_angle1 = a3;
			r_angle2 = a1;
			r_angle3 = a2;
			break;
		default:	// ZERO or ALL toes touching
			r_angle1 = -1.0;
			r_angle2 = -1.0;
			r_angle3 = -1.0;
	}
}

void R_SetMotorsFromState() {
	bFloatDuringInactiveMotorPWM = false;
	switch (r_toeState) {
		case TOE1 | TOE2:
		case TOE1:
			motor[leg1] = r_power1;
			motor[leg2] = r_power2;
			motor[leg3] = r_power3;
			break;
		case TOE2 | TOE3:
		case TOE2:
			motor[leg2] = r_power1;
			motor[leg3] = r_power2;
			motor[leg1] = r_power3;
			break;
		case TOE3 | TOE1:
		case TOE3:
			motor[leg3] = r_power1;
			motor[leg1] = r_power2;
			motor[leg2] = r_power3;
			break;
		default:	// ZERO or ALL toes touching
			motor[leg1] = 0;
			motor[leg2] = 0;
			motor[leg3] = 0;
			bFloatDuringInactiveMotorPWM = true;
	}
}

void R_SetANNInputsFromLegAngle(int eLeg, float angle) {
	int offset;
	switch (eLeg) {
		case LEG1: offset = 0; break;
		case LEG2: offset = 5; break;
		case LEG3: offset = 10; break;
		default: return;
	}
	float center;
	for (int i = 0; i < 5; i++) {
		center = i * 0.25;	// 0, 1/4, 1/2, 3/4, 1
		if (r_isTwoToes) {
			r_ann2.inputs[offset + i] = Gaussian(angle, center);
		} else {
			r_ann1.inputs[offset + i] = Gaussian(angle, center);
		}
	}
}

void R_SetANNInputsFromState() {
	R_SetANNInputsFromLegAngle(LEG1, r_angle1);
	R_SetANNInputsFromLegAngle(LEG2, r_angle2);
	R_SetANNInputsFromLegAngle(LEG3, r_angle3);
}

void R_GetANNInputs(InputArray& inputs) {
	int i;
	if (r_isTwoToes) {
		for (i = 0; i < ANN_NUM_INPUTS; i++) {
			inputs.a[i] = r_ann2.inputs[i];
		}
	} else {
		for (i = 0; i < ANN_NUM_INPUTS; i++) {
			inputs.a[i] = r_ann1.inputs[i];
		}
	}
}

void R_SetANNInputs(InputArray& inputs) {
	int i;
	if (r_isTwoToes) {
		for (i = 0; i < ANN_NUM_INPUTS; i++) {
			r_ann2.inputs[i] = inputs.a[i];
		}
	} else {
		for (i = 0; i < ANN_NUM_INPUTS; i++) {
			r_ann1.inputs[i] = inputs.a[i];
		}
	}
}

float R_Activate() {
	if (r_isTwoToes) {
		Activate(r_ann2);
		return r_ann2.output;
	} else {
		Activate(r_ann1);
		return r_ann1.output;
	}
}

// PRE: SetANNInputsFromState
int R_ChooseLegPower(int eLeg) {
	float curAngle;
	switch (eLeg) {
		case LEG1: curAngle = r_angle1; break;
		case LEG2: curAngle = r_angle2; break;
		case LEG3: curAngle = r_angle3; break;
		default: return 0;
	}
	// Try angles {0.0, 0.1, ..., 0.9, 1.0}
	float bestAngle = curAngle;
	float curActivation = 0.0;
	float bestActivation = 0.0;
	float step = 0.1;
	float testAngle = 0.0;
	for (int i = 0; i <= 10; i++) {
		testAngle = i*step;
		R_SetANNInputsFromLegAngle(eLeg, testAngle);
		curActivation = R_Activate();
		if (curActivation > bestActivation) {
			bestActivation = curActivation;
			bestAngle = testAngle;
		} else if (curActivation == bestActivation) {
			// if all else equal, prefer lower powers
			if (abs(testAngle-curAngle) < abs(bestAngle-curAngle)) {
				bestAngle = testAngle;
			}
		}
	}
	// Choose power proportional to angle difference:
	int power;
	if (bestAngle > curAngle + 0.1) power = MAX_POWER;
	else if (bestAngle < curAngle - 0.1) power = -MAX_POWER;
	else power = (int) ((bestAngle - curAngle) * 10 * MAX_POWER);
	R_SetANNInputsFromLegAngle(eLeg, curAngle);	// reset current state
	return power;
}
int R_ChooseLegPower_old(int eLeg) {
	float curAngle;
	float newAngle;
	float curActivation = 0.0;
	float bestActivation = 0.0;
	int bestPower = 0;
	switch (eLeg) {
		case LEG1: curAngle = r_angle1; break;
		case LEG2: curAngle = r_angle2; break;
		case LEG3: curAngle = r_angle3; break;
		default: return 0;
	}
	// prevent motors from trying to extend leg past bounds:
	int step = 20;
	int minPower = (curAngle < 0.05) ? 0 : -step*3;
	int maxPower = (curAngle > 0.95) ? 0 : step*3;
	for (int power = minPower; power <= maxPower; power += step) {
		newAngle = EstimateAngleChange(curAngle, power);
		R_SetANNInputsFromLegAngle(eLeg, newAngle);
		curActivation = R_Activate();
		if (curActivation > bestActivation) {
			bestActivation = curActivation;
			bestPower = power;
		} else if (curActivation == bestActivation) {
			// if all else equal, prefer lower powers
			if (abs(power) < abs(bestPower)) {
				bestPower = power;
			}
		}
	}
	R_SetANNInputsFromLegAngle(eLeg, curAngle);	// reset current state
	return bestPower;
}

float R_GetRewardSignal(int prevToeState) {
	// common case: toe state has not changed
	if (r_toeState == prevToeState) {
		if (!r_isTrainingMode) {
			// under normal autonomous operation, reward is current state value estimate, decayed
			return DECAY_FACTOR * R_Activate();
		} else {
			// in training mode, reward is constant special training value
			return REWARD_TRAIN;
		}
	}
	// positive reward: next toe down or rear toe up
	if (	((prevToeState == TOE1) && (r_toeState == (TOE1 | TOE2)))
		||	((prevToeState == TOE2) && (r_toeState == (TOE2 | TOE3)))
		||	((prevToeState == TOE3) && (r_toeState == (TOE3 | TOE1)))
		||	((prevToeState == (TOE1 | TOE2)) && (r_toeState == TOE2))
		||	((prevToeState == (TOE2 | TOE3)) && (r_toeState == TOE3))
		||	((prevToeState == (TOE3 | TOE1)) && (r_toeState == TOE1)) )
	{
		return REWARD_MAX;
	}
	// negative reward: we're going backwards!
	if (	((prevToeState == TOE1) && (r_toeState == (TOE3 | TOE1)))
		||	((prevToeState == TOE2) && (r_toeState == (TOE1 | TOE2)))
		||	((prevToeState == TOE3) && (r_toeState == (TOE2 | TOE3)))
		||	((prevToeState == (TOE1 | TOE2)) && (r_toeState == TOE1))
		||	((prevToeState == (TOE2 | TOE3)) && (r_toeState == TOE2))
		||	((prevToeState == (TOE3 | TOE1)) && (r_toeState == TOE3)) )
	{
		return REWARD_MIN;
	}
	// if we get this far, something is wrong with the toe state (maybe robot has been lifted in the air?)
	return NO_REWARD;
}

void R_Train(int prevToeState, float targetValue) {
	if (	(prevToeState == TOE1)
		||	(prevToeState == TOE2)
		||	(prevToeState == TOE3)	)
	{
		if (!r_isTrainingMode) {
			Train(r_ann1, targetValue);
		} else {
			// in training mode, only give positive reinforcement
			TrainPositiveOnly(r_ann1, targetValue);
		}
	}
	else
	if (	(prevToeState == (TOE1 | TOE2))
		||	(prevToeState == (TOE2 | TOE3))
		||	(prevToeState == (TOE3 | TOE1)) )
	{
		if (!r_isTrainingMode) {
			Train(r_ann2, targetValue);
		} else {
			// in training mode, only give positive reinforcement
			TrainPositiveOnly(r_ann2, targetValue);
		}
	}
}

void R_SenseActLearn(RobotSettings& rs) {
	InputArray inputs;
	R_SetStateFromSensors(rs);
	R_SetANNInputsFromState();
	R_GetANNInputs(inputs);
	int curToeState = r_toeState;
	bool curIsTwoToes = r_isTwoToes;
	if (!r_isTrainingMode) {
		if (!r_isHardcodeMode) {
			r_power1 = R_ChooseLegPower(LEG1);
			r_power2 = R_ChooseLegPower(LEG2);
			r_power3 = R_ChooseLegPower(LEG3);
		} else {
			// Special hard-coded algorithm:
			float targetAngle1 = 0.0;
			float targetAngle2 = 0.0;
			float targetAngle3 = 0.0;
			if (r_isTwoToes) {
				targetAngle1 = 0.5; // rear touching leg
				targetAngle2 = 1.0; // front touching leg
				targetAngle3 = 1.0; // leg in the air
			} else {
				targetAngle1 = 0.3; // only touching leg
				targetAngle2 = 0.7; // front falling leg
				targetAngle3 = 0.5; // rear rising leg
			}
			if (targetAngle1> r_angle1 + 0.1) r_power1 = MAX_POWER;
			else if (targetAngle1 < r_angle1 - 0.1) r_power1 = -MAX_POWER;
			else r_power1 = (int) ((targetAngle1 - r_angle1) * 10 * MAX_POWER);
			if (targetAngle2 > r_angle2 + 0.1) r_power2 = MAX_POWER;
			else if (targetAngle2 < r_angle2 - 0.1) r_power2 = -MAX_POWER;
			else r_power2 = (int) ((targetAngle2 - r_angle2) * 10 * MAX_POWER);
			if (targetAngle3 > r_angle3 + 0.1) r_power3 = MAX_POWER;
			else if (targetAngle3 < r_angle3 - 0.1) r_power3 = -MAX_POWER;
			else r_power3 = (int) ((targetAngle3 - r_angle3) * 10 * MAX_POWER);
		}
		R_SetMotorsFromState();
	} else {
		motor[leg1] = 0;
		motor[leg2] = 0;
		motor[leg3] = 0;
		bFloatDuringInactiveMotorPWM = true;
	}
	wait1Msec(STEP_DELAY);
	R_SetStateFromSensors(rs);
	R_SetANNInputsFromState();
	float reward = R_GetRewardSignal(curToeState);
	if (reward != NO_REWARD) {
		//PlayImmediateTone(300 + (int)(reward*900), 8);	// Play tone to indicate reward level
		bool newIsTwoToes = r_isTwoToes;
		r_isTwoToes = curIsTwoToes;
		r_toeState = curToeState;
		R_SetANNInputs(inputs);	// restore previous input state for training
		float estimate = R_Activate();	// required prior to training
		nxtDisplayCenteredTextLine(3, "%d", (int)(1000 * estimate));
		R_Train(curToeState, reward);
		r_isTwoToes = newIsTwoToes;
	}
}

#endif
