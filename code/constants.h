/* CS572 - Robotics
 * Final Project
 * 25 May 2010
 * Nicholas Bone
 */

#ifndef CONSTANTS_H
#define CONSTANTS_H

// macros
#ifndef max
#define max(a,b) (((b) > (a)) ? (b) : (a))
#endif
#ifndef min
#define min(a,b) (((b) < (a)) ? (b) : (a))
#endif

// sounds
#define TONE_NOTIFY 660

// ANN constants
const int ANN_NUM_INPUTS = 15;
const int ANN_NUM_HIDDEN = 3;
//const int ANN_NUM_OUTPUTS = 1;
const float STARTING_THRESHOLD = -1.0;
const float LEARNING_RATE = 0.02;

// bit values for which toes are touching
#define TOE1	1
#define TOE2	2
#define TOE3	4

// logical constants for identifying which leg (NOTE: uses of enum seem to cause compiler crash?)
#define LEG1	1
#define LEG2	2
#define LEG3	3

// robot control constants
const int STEP_DELAY = 50; // milliseconds to wait after taking action before sensing next state
const int MAX_POWER = 75;

// robot training constants
const int DECAY_FACTOR = 0.99;	// TODO: is this a good value?
const float REWARD_MAX = 1.0;		// maximum value a state can have
const float REWARD_MIN = 0.0;		// minimum value a state can have
const float REWARD_TRAIN = 0.50;	// reward given to all states encountered while in special
									// training mode
const float NO_REWARD = -1.0;	// dummy value indicating absence of reward (suppresses training)

// miscellaneous math constants
const float GAUSSIAN_BETA = 4.0;

#endif
