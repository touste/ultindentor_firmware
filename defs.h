#ifndef DEFS_H
#define DEFS_H


#include <math.h>
#include <stdio.h>
#include <stdlib.h>
#include <string.h>
#include <inttypes.h>

#include <avr/interrupt.h>

#include "SerialCommand.h"
#include "fastio.h"
#include "conf.h"
#include "pins.h"

#include "Arduino.h"


#include "XYmotion.h"
#include "Zmotion.h"
#include "Head_selector.h"
#include "SerialRefs.h"



#define  FORCE_INLINE __attribute__((always_inline)) inline

#define CRITICAL_SECTION_START  unsigned char _sreg = SREG; cli();
#define CRITICAL_SECTION_END    SREG = _sreg;

#define  enable_x() {SET_OUTPUT(X_ENABLE_PIN); WRITE(X_ENABLE_PIN, X_ENABLE_ON);}
#define disable_x() {SET_OUTPUT(X_ENABLE_PIN); WRITE(X_ENABLE_PIN, !X_ENABLE_ON);}

#define  enable_y() {SET_OUTPUT(Y_ENABLE_PIN); WRITE(Y_ENABLE_PIN, Y_ENABLE_ON);}
#define disable_y() {SET_OUTPUT(Y_ENABLE_PIN); WRITE(Y_ENABLE_PIN, !Y_ENABLE_ON);}

#define  enable_z() {SET_OUTPUT(Z_ENABLE_P); WRITE(Z_ENABLE_P, Z_ENABLE_ON);}
#define disable_z() {SET_OUTPUT(Z_ENABLE_P); WRITE(Z_ENABLE_P, !Z_ENABLE_ON);} 

#endif







