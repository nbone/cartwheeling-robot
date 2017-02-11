/* CS572 - Robotics
 * Final Project
 * 25 May 2010
 * Nicholas Bone
 */

#ifndef ANN_H
#define ANN_H

#ifndef CONSTANTS_H
#include "constants.h"
#endif

/* NOTE: RobotC generates faulty runtime on some nested structs, so what used to be Neuron objects are now inlined within the ANN */

typedef struct {
	// Net inputs
	float inputs[ANN_NUM_INPUTS];
	// Hidden node 1
	float h1w[ANN_NUM_INPUTS + 1];	// 1-N+1, where wN+1 is threshold
	float h1out;	// output of node
	float h1delta;	// node delta for backprop training
	// Hidden node 2
	float h2w[ANN_NUM_INPUTS + 1];
	float h2out;
	float h2delta;
	// Hidden node 3
	float h3w[ANN_NUM_INPUTS + 1];
	float h3out;
	float h3delta;
	// Output node
	float o1w[ANN_NUM_HIDDEN + 1];
	float o1out;
	float o1delta;
	// Net output
	float output;
} ANN;

typedef struct {
	// Net inputs
	float inputs[ANN_NUM_INPUTS];
	// Weights
	float w[ANN_NUM_INPUTS];
	// Net output
	float output;
} LMS;
#define LMS_MULTIPLIER	100

/* NOTE: RobotC sometimes gives a runtime error on the exp function (for no good reason), so implement Sigmoid using a lookup table */
typedef struct {
	float sigmoidTenths[100]; // sigmoid values for inputs from 0.0 to 9.9
	float sigmoidInputMax;
} SigmoidLookupTable;
SigmoidLookupTable g_sigmoidLookupTable;

void InitSigmoidLookupTable();

float Sigmoid(float f);

float SmallRandomFloat();

float TimesOneMinus(float f);

float SigmoidGradientError(float actual, float desired);

/* NOTE: RobotC compiler seems to generate faulty runtime for these, so inlined them in Init(ANN) instead:
void Init(NeuronHid& n);
void Init(NeuronOut& n);
*/

void Init(ANN& ann);

/* NOTE: Inlined these in Activate(ANN) on suspicion that RobotC compiler generates faulty runtime:
void Activate(NeuronHid& n);
void Activate(NeuronOut& n);
*/

void Activate(ANN& ann);

void Train(ANN& ann, float target);

void TrainPositiveOnly(ANN& ann, float target);


#endif
