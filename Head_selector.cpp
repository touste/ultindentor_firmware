// 
// 
// 

#include "Head_selector.h"
#include "defs.h"

Servo headselector;
extern SerialCommand SCmd;

void SelectProbe(int numProbe)
{

	WRITE(ENABLE_SERVO_PIN, LOW);
	headselector.attach(HEAD_SERVO_PIN);

	switch(numProbe) {
		//2370
	case 1 : headselector.writeMicroseconds(1925);
			 WRITE(ENABLE_LC1_PIN, LOW);
			 WRITE(ENABLE_LC2_PIN, HIGH);
			 break;
	case 2 : headselector.writeMicroseconds(1150);
     		 WRITE(ENABLE_LC1_PIN, HIGH);
			 WRITE(ENABLE_LC2_PIN, LOW);
		     break;
    case 3 : headselector.writeMicroseconds(2200);
		     break;
	}

	delay(800);

	headselector.detach();

	WRITE(HEAD_SERVO_PIN, LOW);

	WRITE(ENABLE_SERVO_PIN, HIGH);

	

}



