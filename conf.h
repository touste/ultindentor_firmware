#ifndef CONF_H
#define CONF_H

#include "pins.h"

//#define DEBUG


#define MAX_SPEED_XY 100
#define MIN_SPEED_XY 2 

#define MAX_SPEED_Z 120
#define MIN_SPEED_Z 0.01

#define F_CPU 16000000

#define INTERRUPT_FREQ_XY 2283217

#define INTERRUPT_FREQ_Z 1455333

#define AXIS_STEPS_PER_UNIT_XY 78.7402

#define MAX_DIST_X 124.50
#define MAX_DIST_Y 158.10
#define MAX_DIST_Z 140.50

#define AXIS_STEPS_PER_UNIT_Z 1000.0

#define ACCEL_XY 500
#define JERK_XY 10000

#define ACCEL_Z 3700
#define HARDSTOP_DESCEL_Z 50000
#define JERK_Z 500000

#define CPUCYCLES_Z 1

#define CPUCYCLES_XY 1

#define SCURVES_XY true
#define SCURVES_Z false

#define X_ENABLE_ON false
#define Y_ENABLE_ON false
#define Z_ENABLE_ON false

#define FORWARD_X true
#define FORWARD_Y false
#define FORWARD_Z true

#define HOMING_SPEED_XY 60
#define HOMING_SPEED_Z 30

#define MIN_MOTION_XY 4.0/AXIS_STEPS_PER_UNIT_XY

#define MIN_MOTION_Z 10.0/AXIS_STEPS_PER_UNIT_Z

const bool X_MIN_ENDSTOP_INVERTING = true; // set to true to invert the logic of the endstop.
const bool Y_MIN_ENDSTOP_INVERTING = true; // set to true to invert the logic of the endstop.
const bool Z_MIN_ENDSTOP_INVERTING = true; // set to true to invert the logic of the endstop.
const bool X_MAX_ENDSTOP_INVERTING = true; // set to true to invert the logic of the endstop.
const bool Y_MAX_ENDSTOP_INVERTING = true; // set to true to invert the logic of the endstop.
const bool Z_MAX_ENDSTOP_INVERTING = true; // set to true to invert the logic of the endstop.

#endif
