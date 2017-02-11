/* CS572 - Robotics
 * Final Project
 * 25 May 2010
 * Nicholas Bone
 */

#ifndef ANN_C
#define ANN_C

#ifndef ANN_H
#include "ann.h"
#endif


void InitSigmoidLookupTable() {
	g_sigmoidLookupTable.sigmoidInputMax = 9.9;
	g_sigmoidLookupTable.sigmoidTenths[0] = 0.5;
	g_sigmoidLookupTable.sigmoidTenths[1] = 0.475020813;
	g_sigmoidLookupTable.sigmoidTenths[2] = 0.450166003;
	g_sigmoidLookupTable.sigmoidTenths[3] = 0.425557483;
	g_sigmoidLookupTable.sigmoidTenths[4] = 0.40131234;
	g_sigmoidLookupTable.sigmoidTenths[5] = 0.377540669;
	g_sigmoidLookupTable.sigmoidTenths[6] = 0.354343694;
	g_sigmoidLookupTable.sigmoidTenths[7] = 0.331812228;
	g_sigmoidLookupTable.sigmoidTenths[8] = 0.310025519;
	g_sigmoidLookupTable.sigmoidTenths[9] = 0.289050497;
	g_sigmoidLookupTable.sigmoidTenths[10] = 0.268941421;
	g_sigmoidLookupTable.sigmoidTenths[11] = 0.249739894;
	g_sigmoidLookupTable.sigmoidTenths[12] = 0.231475217;
	g_sigmoidLookupTable.sigmoidTenths[13] = 0.214165017;
	g_sigmoidLookupTable.sigmoidTenths[14] = 0.197816111;
	g_sigmoidLookupTable.sigmoidTenths[15] = 0.182425524;
	g_sigmoidLookupTable.sigmoidTenths[16] = 0.167981615;
	g_sigmoidLookupTable.sigmoidTenths[17] = 0.154465265;
	g_sigmoidLookupTable.sigmoidTenths[18] = 0.141851065;
	g_sigmoidLookupTable.sigmoidTenths[19] = 0.130108474;
	g_sigmoidLookupTable.sigmoidTenths[20] = 0.119202922;
	g_sigmoidLookupTable.sigmoidTenths[21] = 0.109096821;
	g_sigmoidLookupTable.sigmoidTenths[22] = 0.099750489;
	g_sigmoidLookupTable.sigmoidTenths[23] = 0.091122961;
	g_sigmoidLookupTable.sigmoidTenths[24] = 0.083172696;
	g_sigmoidLookupTable.sigmoidTenths[25] = 0.07585818;
	g_sigmoidLookupTable.sigmoidTenths[26] = 0.06913842;
	g_sigmoidLookupTable.sigmoidTenths[27] = 0.062973356;
	g_sigmoidLookupTable.sigmoidTenths[28] = 0.057324176;
	g_sigmoidLookupTable.sigmoidTenths[29] = 0.052153563;
	g_sigmoidLookupTable.sigmoidTenths[30] = 0.047425873;
	g_sigmoidLookupTable.sigmoidTenths[31] = 0.043107255;
	g_sigmoidLookupTable.sigmoidTenths[32] = 0.039165723;
	g_sigmoidLookupTable.sigmoidTenths[33] = 0.035571189;
	g_sigmoidLookupTable.sigmoidTenths[34] = 0.032295465;
	g_sigmoidLookupTable.sigmoidTenths[35] = 0.029312231;
	g_sigmoidLookupTable.sigmoidTenths[36] = 0.026596994;
	g_sigmoidLookupTable.sigmoidTenths[37] = 0.024127021;
	g_sigmoidLookupTable.sigmoidTenths[38] = 0.021881271;
	g_sigmoidLookupTable.sigmoidTenths[39] = 0.019840306;
	g_sigmoidLookupTable.sigmoidTenths[40] = 0.01798621;
	g_sigmoidLookupTable.sigmoidTenths[41] = 0.016302499;
	g_sigmoidLookupTable.sigmoidTenths[42] = 0.014774032;
	g_sigmoidLookupTable.sigmoidTenths[43] = 0.013386918;
	g_sigmoidLookupTable.sigmoidTenths[44] = 0.012128435;
	g_sigmoidLookupTable.sigmoidTenths[45] = 0.010986943;
	g_sigmoidLookupTable.sigmoidTenths[46] = 0.009951802;
	g_sigmoidLookupTable.sigmoidTenths[47] = 0.009013299;
	g_sigmoidLookupTable.sigmoidTenths[48] = 0.008162571;
	g_sigmoidLookupTable.sigmoidTenths[49] = 0.007391541;
	g_sigmoidLookupTable.sigmoidTenths[50] = 0.006692851;
	g_sigmoidLookupTable.sigmoidTenths[51] = 0.006059801;
	g_sigmoidLookupTable.sigmoidTenths[52] = 0.005486299;
	g_sigmoidLookupTable.sigmoidTenths[53] = 0.004966802;
	g_sigmoidLookupTable.sigmoidTenths[54] = 0.004496273;
	g_sigmoidLookupTable.sigmoidTenths[55] = 0.004070138;
	g_sigmoidLookupTable.sigmoidTenths[56] = 0.00368424;
	g_sigmoidLookupTable.sigmoidTenths[57] = 0.003334807;
	g_sigmoidLookupTable.sigmoidTenths[58] = 0.003018416;
	g_sigmoidLookupTable.sigmoidTenths[59] = 0.002731961;
	g_sigmoidLookupTable.sigmoidTenths[60] = 0.002472623;
	g_sigmoidLookupTable.sigmoidTenths[61] = 0.002237849;
	g_sigmoidLookupTable.sigmoidTenths[62] = 0.00202532;
	g_sigmoidLookupTable.sigmoidTenths[63] = 0.001832939;
	g_sigmoidLookupTable.sigmoidTenths[64] = 0.001658801;
	g_sigmoidLookupTable.sigmoidTenths[65] = 0.001501182;
	g_sigmoidLookupTable.sigmoidTenths[66] = 0.00135852;
	g_sigmoidLookupTable.sigmoidTenths[67] = 0.001229399;
	g_sigmoidLookupTable.sigmoidTenths[68] = 0.001112536;
	g_sigmoidLookupTable.sigmoidTenths[69] = 0.001006771;
	g_sigmoidLookupTable.sigmoidTenths[70] = 0.000911051;
	g_sigmoidLookupTable.sigmoidTenths[71] = 0.000824425;
	g_sigmoidLookupTable.sigmoidTenths[72] = 0.000746029;
	g_sigmoidLookupTable.sigmoidTenths[73] = 0.000675083;
	g_sigmoidLookupTable.sigmoidTenths[74] = 0.000610879;
	g_sigmoidLookupTable.sigmoidTenths[75] = 0.000552779;
	g_sigmoidLookupTable.sigmoidTenths[76] = 0.000500201;
	g_sigmoidLookupTable.sigmoidTenths[77] = 0.000452622;
	g_sigmoidLookupTable.sigmoidTenths[78] = 0.000409567;
	g_sigmoidLookupTable.sigmoidTenths[79] = 0.000370606;
	g_sigmoidLookupTable.sigmoidTenths[80] = 0.00033535;
	g_sigmoidLookupTable.sigmoidTenths[81] = 0.000303447;
	g_sigmoidLookupTable.sigmoidTenths[82] = 0.000274578;
	g_sigmoidLookupTable.sigmoidTenths[83] = 0.000248455;
	g_sigmoidLookupTable.sigmoidTenths[84] = 0.000224817;
	g_sigmoidLookupTable.sigmoidTenths[85] = 0.000203427;
	g_sigmoidLookupTable.sigmoidTenths[86] = 0.000184072;
	g_sigmoidLookupTable.sigmoidTenths[87] = 0.000166558;
	g_sigmoidLookupTable.sigmoidTenths[88] = 0.00015071;
	g_sigmoidLookupTable.sigmoidTenths[89] = 0.00013637;
	g_sigmoidLookupTable.sigmoidTenths[90] = 0.000123395;
	g_sigmoidLookupTable.sigmoidTenths[91] = 0.000111653;
	g_sigmoidLookupTable.sigmoidTenths[92] = 0.000101029;
	g_sigmoidLookupTable.sigmoidTenths[93] = 0.0000914159;
	g_sigmoidLookupTable.sigmoidTenths[94] = 0.0000827172;
	g_sigmoidLookupTable.sigmoidTenths[95] = 0.0000748462;
	g_sigmoidLookupTable.sigmoidTenths[96] = 0.0000677241;
	g_sigmoidLookupTable.sigmoidTenths[97] = 0.0000612797;
	g_sigmoidLookupTable.sigmoidTenths[98] = 0.0000554485;
	g_sigmoidLookupTable.sigmoidTenths[99] = 0.0000501722;
}


float Sigmoid(float f) {
	const float mult = 2.0; // steepness of S-curve
	f *= mult;
	if (f > g_sigmoidLookupTable.sigmoidInputMax) {
		return 1.0;
	}
	if (f < -g_sigmoidLookupTable.sigmoidInputMax) {
		return 0.0;
	}
	bool isNeg = false;
	if (f < 0) {
		f = 0.0 - f;
		isNeg = true;
	}
	float f10 = 10.0 * f;
	int indexLow = (int)f10;
	int indexHigh = (int)(f10+1);
	float result = (indexHigh - f10)*g_sigmoidLookupTable.sigmoidTenths[indexLow] + (f10 - indexLow)*g_sigmoidLookupTable.sigmoidTenths[indexHigh];
	if (!isNeg) {
		result = 1.0 - result;
	}
	return result;
//	return 1.0 / (1.0 + exp(-f));
}

float SmallRandomFloat() {
	return -0.1 + ((rand() % 100) / 500.0);
}

float TimesOneMinus(float f) {
	return f * (1 - f);
}

float SigmoidGradientError(float actual, float desired) {
	return TimesOneMinus(actual) * (desired - actual);
}

void Init(ANN& ann) {
	for (int inp = 0; inp < ANN_NUM_INPUTS; inp++) {
		ann.h1w[inp] = 0.0;
		ann.h2w[inp] = 0.0;
		ann.h3w[inp] = 0.0;
	}
	for (int inp = 0; inp < ANN_NUM_INPUTS; inp++) {
		ann.inputs[inp] = 0.0;
		ann.h1w[inp%5] = SmallRandomFloat();
		ann.h2w[inp%5+5] = SmallRandomFloat();
		ann.h3w[inp%5+10] = SmallRandomFloat();
	}
	ann.h1w[ANN_NUM_INPUTS] = STARTING_THRESHOLD;
	ann.h2w[ANN_NUM_INPUTS] = STARTING_THRESHOLD;
	ann.h3w[ANN_NUM_INPUTS] = STARTING_THRESHOLD;
	ann.h1out = 0.0;
	ann.h2out = 0.0;
	ann.h3out = 0.0;
	ann.h1delta = 0.0;
	ann.h2delta = 0.0;
	ann.h3delta = 0.0;
	for (int i = 0; i < ANN_NUM_HIDDEN; i++) {
		ann.o1w[i] = SmallRandomFloat();
	}
	ann.o1w[ANN_NUM_HIDDEN] = STARTING_THRESHOLD;
	ann.o1out = 0.0;
	ann.o1delta = 0.0;
	ann.output = 0.0;
}

void Init(LMS& lms) {
	for (int inp = 0; inp < ANN_NUM_INPUTS; inp++) {
		lms.inputs[inp] = 0.0;
		lms.w[inp] = SmallRandomFloat();
	}
	lms.output = 0.0;
}

void Activate(ANN& ann) {
	float sum;
	int i;
	ann.output = 0.0;
	// activate hidden node 1
	sum = ann.h1w[ANN_NUM_INPUTS];
	for (i = 0; i < 5; i++) {
		sum += ann.h1w[i] * ann.inputs[i];
	}
	ann.h1out = Sigmoid(sum);
	// activate hidden node 2
	sum = ann.h2w[ANN_NUM_INPUTS];
	for (i = 5; i < 10; i++) {
		sum += ann.h2w[i] * ann.inputs[i];
	}
	ann.h2out = Sigmoid(sum);
	// activate hidden node 3
	sum = ann.h3w[ANN_NUM_INPUTS];
	for (i = 10; i < 15; i++) {
		sum += ann.h3w[i] * ann.inputs[i];
	}
	ann.h3out = Sigmoid(sum);
	// activate output node
	sum = ann.o1w[ANN_NUM_HIDDEN];
	sum += ann.o1w[0] * ann.h1out;
	sum += ann.o1w[1] * ann.h2out;
	sum += ann.o1w[2] * ann.h3out;
	ann.o1out = Sigmoid(sum);
	ann.output = ann.o1out;
}

void Activate(LMS& lms) {
	lms.output = 0.0;
	for (int i = 0; i < ANN_NUM_INPUTS; i++) {
		lms.output += lms.w[i] * lms.inputs[i];
	}
	lms.output /= LMS_MULTIPLIER;
}

void Train(ANN& ann, float target) {
	// ASSUME that ann's output (and ann's neuron's inputs/outputs) still hold the values we want to adjust.
	// Calculate error gradient for output layer:
	ann.o1delta = SigmoidGradientError(ann.output, target);
	// Propagate back to hidden layer:
	ann.h1delta = ann.o1w[0] * ann.o1delta * TimesOneMinus(ann.h1out);
	ann.h2delta = ann.o1w[1] * ann.o1delta * TimesOneMinus(ann.h2out);
	ann.h3delta = ann.o1w[2] * ann.o1delta * TimesOneMinus(ann.h3out);
	// Update output layer weights:
	ann.o1w[0] += LEARNING_RATE * ann.o1delta * ann.h1out;
	ann.o1w[1] += LEARNING_RATE * ann.o1delta * ann.h2out;
	ann.o1w[2] += LEARNING_RATE * ann.o1delta * ann.h3out;
	ann.o1w[ANN_NUM_HIDDEN] += LEARNING_RATE * ann.o1delta;
	// Update hidden layer weights:
	int i;
	for (i = 0; i < 5; i++) {
		ann.h1w[i] += LEARNING_RATE * ann.h1delta * ann.inputs[i];
	}
	for (int i = 5; i < 10; i++) {
		ann.h2w[i] += LEARNING_RATE * ann.h2delta * ann.inputs[i];
	}
	for (int i = 10; i < 15; i++) {
		ann.h3w[i] += LEARNING_RATE * ann.h3delta * ann.inputs[i];
	}
	ann.h1w[ANN_NUM_INPUTS] += LEARNING_RATE * ann.h1delta;
	ann.h2w[ANN_NUM_INPUTS] += LEARNING_RATE * ann.h2delta;
	ann.h3w[ANN_NUM_INPUTS] += LEARNING_RATE * ann.h3delta;
}

void Train(LMS& lms, float target) {
	int i;
	// Training signal is desired output minus actual output:
	float delta = (target * LMS_MULTIPLIER) - lms.output;
	// Sum the inputs for normalization of the weight update:
	float sumInputs = 0.0;
	for (i = 0; i < ANN_NUM_INPUTS; i++) {
		sumInputs += lms.inputs[i];
	}
	// Update the weights:
	for (i = 0; i < ANN_NUM_INPUTS; i++) {
		lms.w[i] += LEARNING_RATE * delta * lms.inputs[i] / sumInputs;
	}
}

void TrainPositiveOnly(ANN& ann, float target) {
	if (target <= ann.output) {
		return;
	}
	Train(ann, target);
}

void TrainPositiveOnly(LMS& lms, float target) {
	if ((target * LMS_MULTIPLIER) <= lms.output) {
		return;
	}
	Train(lms, target);
}

#endif
