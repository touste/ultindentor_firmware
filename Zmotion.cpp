// 
// 
// 

#include "Zmotion.h"
#include "defs.h"
#include "gen_tcounts_curves.h"
#include "microsmooth.h"


unsigned int curve_table_z[1650];
unsigned int curve_table_z_hardstop[150];
motion_z motionz;
extern SerialCommand SCmd;



FORCE_INLINE void quickStop_z()  // stop now! and reinitialize everything
{
	disable_z();
	Z_init();
}


FORCE_INLINE void checkHitEndstops_z()  // check if the endstops are activated. if homing, do not completely stop the motion (stop only the concerned axis)
{
	if ((motionz.direction_z==1) && (READ(Z_MAX_PIN) != Z_MAX_ENDSTOP_INVERTING))
	{
		quickStop_z();
		motionz.touchswith_z_max = true;
		return;
	}
	else if ((motionz.direction_z==-1) && (READ(Z_MIN_PIN) != Z_MIN_ENDSTOP_INVERTING))
	{
		quickStop_z();
		motionz.touchswith_z_min = true;
		return;
	}

}



void home_Z()
{
	moveZ_rel(-600, HOMING_SPEED_Z, false);  // arbitrary values, just to be sure home is reached
	Z_init_absolute();  // initialize everything
}


void moveZ_abs(float z, float speed, bool hardstop, bool get_contact, float indent_after_contact)
{
	moveZ_rel(z - motionz.pos_z, speed, hardstop, get_contact, indent_after_contact);  //  transform an absolute motion to a relative one
}



void Z_init()  //  initialize all motion variables
{

  motionz.rel_steps_z = 0;

  motionz.step_accel_z = 0;
  motionz.step_descel_z = 0;
  motionz.step_constant_z = 0;

  motionz.direction_z = 0;

  motionz.nominal_speed_z = 0;

  motionz.touchswith_z_min = false;
  motionz.touchswith_z_max = false;

  WRITE(START_ACQ_PIN,LOW);

  //disable_z();
}


void Z_init_absolute()  // same as above, nut with the absolute positions as well
{

  Z_init();

  motionz.pos_z = 0;

}


void moveZ_rel(float z, float speed, bool hardstop, bool get_contact, float indent_after_contact)  // we need to fill all these motion variables!
{
	
	Z_init();  // the homing function has already initialized the variables and activated motionz.homing


	// the following pre-defines the directions based on the sign of z
	if (fabs(z)<MIN_MOTION_Z)
	{
		motionz.direction_z = 0;
		z = 0;
	}
	else if (z>0) 
	{
		motionz.direction_z = 1;
	}
	else if (z<0)
	{
		motionz.direction_z = -1;
	}

	set_directions_z();  // pull the pins depending on the directions


	speed = fabs(speed);
	speed = constrain(speed, MIN_SPEED_Z, MAX_SPEED_Z);  // no more, no less


	unsigned long indent_step = 0;
	unsigned long indent_step_constant = 0;
	unsigned long actual_steps = 0;


	// the following calculates the steps.
	if ((z==0) || (speed==0))  
	{
		Z_init();
		sei();
		return; // no motion
	}
	else  // motion
	{
		motionz.rel_steps_z = (unsigned long) (fabs(z) * (float)AXIS_STEPS_PER_UNIT_Z);

		indent_step = (unsigned long) (fabs(indent_after_contact) * (float)AXIS_STEPS_PER_UNIT_Z);
		
		if (speed != motionz.last_speed_z) {
			motionz.step_accel_z = (unsigned long)gen_timercounts(curve_table_z, speed, INTERRUPT_FREQ_Z, AXIS_STEPS_PER_UNIT_Z, JERK_Z, ACCEL_Z, CPUCYCLES_Z, SCURVES_Z); //The calculation of the speed curve is done now (it takes some time!)
		}
		else
		{
			motionz.step_accel_z = motionz.last_step_accel_z;
		}

		if (motionz.step_accel_z >= (unsigned long) ((float)motionz.rel_steps_z/2.0)) {
			motionz.step_accel_z = (unsigned long) ((float)motionz.rel_steps_z/2.0);
		}

		if (hardstop == true) {
			if (motionz.last_hardstop_z == false) {
				motionz.step_descel_z = min(gen_timercounts(curve_table_z_hardstop, speed, INTERRUPT_FREQ_Z, AXIS_STEPS_PER_UNIT_Z, JERK_Z, HARDSTOP_DESCEL_Z, CPUCYCLES_Z, false), motionz.step_accel_z); //The calculation of the speed curve is done now (it takes some time
			}
			else {
				motionz.step_descel_z = motionz.last_step_descel_hardstop_z;
			}
		}
		else {
			motionz.step_descel_z = motionz.step_accel_z;
		}

		motionz.step_constant_z = motionz.rel_steps_z - motionz.step_accel_z - motionz.step_descel_z;

		motionz.last_speed_z = speed;
		motionz.last_step_accel_z = motionz.step_accel_z;
		motionz.last_hardstop_z = hardstop;
		motionz.last_step_descel_hardstop_z = motionz.step_descel_z;

		if (get_contact == true  && ((indent_step<=motionz.step_descel_z) || (indent_step>=(motionz.step_accel_z+motionz.step_constant_z)))) {
			Z_init();
			sei();
			return; // no motion
		}

		indent_step_constant = indent_step-motionz.step_descel_z;

		enable_z();
	}

	init_endstops_z();  // pull the endstop pins

	actual_steps = motionz.step_constant_z;

	float dcount_min = 1.0/(float)AXIS_STEPS_PER_UNIT_Z*(float)INTERRUPT_FREQ_Z/speed;
	unsigned int constantspeed_dc = (unsigned int) dcount_min - (unsigned int) CPUCYCLES_Z;

	


	if (constantspeed_dc<5) { // I don't know what went wrong but this is bad
		disable_z();
		Z_init();
		sei();
		return;
	}

	// Start the motion:
	#ifdef DEBUG
		Serial.print("Initial absolute z position: ");
		Serial.print(motionz.pos_z);
		Serial.print("\nThe Z motion is starting:\nRelative motion of ");
		Serial.print(z);
		Serial.print(" mm at a speed of ");
		Serial.print(speed);
		Serial.print("\nThis amounts to:\nTotal number of steps: ");
		Serial.print(motionz.rel_steps_z);
		Serial.print(".\nAcceleration steps: ");
		Serial.print(motionz.step_accel_z);
		Serial.print(".\nConstant speed steps: ");
		Serial.print(motionz.step_constant_z);
		Serial.print(".\nDesceleration steps: ");
		Serial.print(motionz.step_descel_z);
		Serial.print(".\nConstant speed dc: ");
		Serial.print(constantspeed_dc);
		Serial.print(".\n\n");
	#endif


    


	Serial.print("GO!\n");
	//delay(1000);
	cli(); //disable interrupts
	


	
	////For timing the loops:
	//    for (unsigned long i=0; i<2000; i++) {
	//	curve_table_z[i] = 65535;
	//}
	//
	////Constant speed phase
	//for (unsigned long i=0; i<1000; i++) {
	//		WRITE(Z_STEP_P, HIGH);  // step if the number of interrupts cycles is greater than a threshold
	//		checkHitEndstops_z();
	//		WRITE(Z_STEP_P,LOW);  // after 1us, pull the pin low and increment the step counter
	//		for (unsigned int j=0; j<curve_table_z[5]; j++) {
	//			__asm__("nop\n\t");
	//		}
	//}


	if (get_contact==false) {

		if (hardstop == false) {

				//Acceleration phase
				for (unsigned long i=0; i<motionz.step_accel_z; i++) {
						WRITE(Z_STEP_P, HIGH);  // step if the number of interrupts cycles is greater than a threshold
						checkHitEndstops_z();
						WRITE(Z_STEP_P,LOW);  // after 1us, pull the pin low and increment the step counter
						for (unsigned int j=0; j<curve_table_z[i]; j++) {
							__asm__("nop\n\t");
						}
				}




				//Constant speed phase
				for (unsigned long i=0; i<motionz.step_constant_z; i++) {
						WRITE(Z_STEP_P, HIGH);  // step if the number of interrupts cycles is greater than a threshold
						checkHitEndstops_z();
						WRITE(Z_STEP_P,LOW);  // after 1us, pull the pin low and increment the step counter
						for (unsigned int j=0; j<constantspeed_dc; j++) {
							__asm__("nop\n\t");
						}
				}

		
				//Desceleration phase
				for (unsigned long i=motionz.step_descel_z; i>0; i--) {
						WRITE(Z_STEP_P, HIGH);  // step if the number of interrupts cycles is greater than a threshold
						checkHitEndstops_z();
						WRITE(Z_STEP_P,LOW);  // after 1us, pull the pin low and increment the step counter
						for (unsigned int j=0; j<curve_table_z[i-1]; j++) {
							__asm__("nop\n\t");
						}
				}

			}

			else {

				//Acceleration phase
				for (unsigned long i=0; i<motionz.step_accel_z; i++) {
						WRITE(Z_STEP_P, HIGH);  // step if the number of interrupts cycles is greater than a threshold
						checkHitEndstops_z();
						WRITE(Z_STEP_P,LOW);  // after 1us, pull the pin low and increment the step counter
						for (unsigned int j=0; j<curve_table_z[i]; j++) {
							__asm__("nop\n\t");
						}
				}


				//Constant speed phase
				for (unsigned long i=0; i<motionz.step_constant_z; i++) {
						WRITE(Z_STEP_P, HIGH);  // step if the number of interrupts cycles is greater than a threshold
						checkHitEndstops_z();
						WRITE(Z_STEP_P,LOW);  // after 1us, pull the pin low and increment the step counter
						for (unsigned int j=0; j<constantspeed_dc; j++) {
							__asm__("nop\n\t");
						}
				}

		
				//Desceleration phase
				for (unsigned long i=motionz.step_descel_z; i>0; i--) {
						WRITE(Z_STEP_P, HIGH);  // step if the number of interrupts cycles is greater than a threshold
						checkHitEndstops_z();
						WRITE(Z_STEP_P,LOW);  // after 1us, pull the pin low and increment the step counter
						for (unsigned int j=0; j<curve_table_z_hardstop[i-1]; j++) {
							__asm__("nop\n\t");
						}
				}

			}

		
	}

	else {

		if (hardstop == false) {

				//Acceleration phase
				for (unsigned long i=0; i<motionz.step_accel_z; i++) {
						WRITE(Z_STEP_P, HIGH);  // step if the number of interrupts cycles is greater than a threshold
						checkHitEndstops_z();
						WRITE(Z_STEP_P,LOW);  // after 1us, pull the pin low and increment the step counter
						for (unsigned int j=0; j<curve_table_z[i]; j++) {
							__asm__("nop\n\t");
						}
				}




				//Constant speed phase before indent
				for (unsigned long i=0; i<motionz.step_constant_z; i++) {
						WRITE(Z_STEP_P, HIGH);  // step if the number of interrupts cycles is greater than a threshold
						if ((motionz.direction_z==1) && (READ(CONTACT_PIN) == true))
						{
							WRITE(Z_STEP_P,LOW);
							WRITE(START_ACQ_PIN, HIGH);
							//Constant speed phase during indent
							for (unsigned long ii=0; ii<indent_step_constant; ii++) {
									WRITE(Z_STEP_P, HIGH);  // step if the number of interrupts cycles is greater than a threshold
									checkHitEndstops_z();
									WRITE(Z_STEP_P,LOW);  // after 1us, pull the pin low and increment the step counter
									for (unsigned int j=0; j<constantspeed_dc; j++) {
										__asm__("nop\n\t");
									}
							}
							actual_steps = i+1 + indent_step_constant;
							break;
						}
						else if ((motionz.direction_z==1) && (READ(Z_MAX_PIN) != Z_MAX_ENDSTOP_INVERTING))
						{
							quickStop_z();
							motionz.touchswith_z_max = true;
							return;
						}
						WRITE(Z_STEP_P,LOW);  // after 1us, pull the pin low and increment the step counter
						for (unsigned int j=0; j<constantspeed_dc; j++) {
							__asm__("nop\n\t");
						}
				}
		
				//Desceleration phase
				for (unsigned long i=motionz.step_descel_z; i>0; i--) {
						WRITE(Z_STEP_P, HIGH);  // step if the number of interrupts cycles is greater than a threshold
						checkHitEndstops_z();
						WRITE(Z_STEP_P,LOW);  // after 1us, pull the pin low and increment the step counter
						for (unsigned int j=0; j<curve_table_z[i-1]; j++) {
							__asm__("nop\n\t");
						}
				}

			}

			else {

				//Acceleration phase
				for (unsigned long i=0; i<motionz.step_accel_z; i++) {
						WRITE(Z_STEP_P, HIGH);  // step if the number of interrupts cycles is greater than a threshold
						checkHitEndstops_z();
						WRITE(Z_STEP_P,LOW);  // after 1us, pull the pin low and increment the step counter
						for (unsigned int j=0; j<curve_table_z[i]; j++) {
							__asm__("nop\n\t");
						}
				}

				//Constant speed phase before indent
				for (unsigned long i=0; i<motionz.step_constant_z; i++) {
						WRITE(Z_STEP_P, HIGH);  // step if the number of interrupts cycles is greater than a threshold
						if ((motionz.direction_z==1) && (READ(CONTACT_PIN) == true))
						{
							WRITE(Z_STEP_P,LOW);
							WRITE(START_ACQ_PIN, HIGH);
							//Constant speed phase during indent
							for (unsigned long ii=0; ii<indent_step_constant; ii++) {
									WRITE(Z_STEP_P, HIGH);  // step if the number of interrupts cycles is greater than a threshold
									checkHitEndstops_z();
									WRITE(Z_STEP_P,LOW);  // after 1us, pull the pin low and increment the step counter
									for (unsigned int j=0; j<constantspeed_dc; j++) {
										__asm__("nop\n\t");
									}
							}
							actual_steps = i+1 + indent_step_constant;
							break;
						}
						else if ((motionz.direction_z==1) && (READ(Z_MAX_PIN) != Z_MAX_ENDSTOP_INVERTING))
						{
							quickStop_z();
							motionz.touchswith_z_max = true;
							return;
						}
						WRITE(Z_STEP_P,LOW);  // after 1us, pull the pin low and increment the step counter
						for (unsigned int j=0; j<constantspeed_dc; j++) {
							__asm__("nop\n\t");
						}
				}

		
				//Desceleration phase
				for (unsigned long i=motionz.step_descel_z; i>0; i--) {
						WRITE(Z_STEP_P, HIGH);  // step if the number of interrupts cycles is greater than a threshold
						checkHitEndstops_z();
						WRITE(Z_STEP_P,LOW);  // after 1us, pull the pin low and increment the step counter
						for (unsigned int j=0; j<curve_table_z_hardstop[i-1]; j++) {
							__asm__("nop\n\t");
						}
				}

			}

			WRITE(START_ACQ_PIN, HIGH);



	}





	sei(); //reenable interrupts
	//Serial.print("STOP!\n");


	if (motionz.touchswith_z_max == true) {
		motionz.pos_z = MAX_DIST_Z;
	}
	else if (motionz.touchswith_z_min == true) {
		motionz.pos_z = 0.0;
	}
	else if (get_contact == true) {
		unsigned long steps = motionz.step_accel_z + actual_steps + motionz.step_descel_z;
		motionz.pos_z = motionz.pos_z + (float)steps*1.0/AXIS_STEPS_PER_UNIT_Z;;
	}
	else {
		motionz.pos_z = motionz.pos_z + z;
	}

	#ifdef DEBUG
		Serial.print("Final absolute z position: ");
		Serial.print(motionz.pos_z);
		Serial.print(".\n\n");
	#endif

	Z_init();



	
	


}


void set_directions_z()  // pull pins depending on the directions of the motion
{
	SET_OUTPUT(Z_DIR_P);


	if (motionz.direction_z==1) 
	{
		WRITE(Z_DIR_P, FORWARD_Z);
	}
	else if (motionz.direction_z==-1)
	{
		WRITE(Z_DIR_P, !FORWARD_Z);
	}

}

void init_endstops_z()
{
	//endstops and pullups

    SET_INPUT(Z_MIN_PIN);
    WRITE(Z_MIN_PIN,HIGH);

    SET_INPUT(Z_MAX_PIN);
    WRITE(Z_MAX_PIN,HIGH);

}






float find_surface_lc(int threshold) {

	unsigned long steps=0;

	int reading = 0;

	Z_init();  // the homing function has already initialized the variables and activated motionz.homing

	motionz.direction_z = 1;

	set_directions_z();  // pull the pins depending on the directions

	enable_z();

	WRITE(ENABLE_ANALOG_PIN, LOW);
	delay(200);


	cli(); //disable interrupts

	unsigned int zero_reading = 0;
	unsigned long total=0;
	float dc = 1000;

	init_sma(0);

	while ((unsigned int)dc>10) {
		WRITE(Z_STEP_P, HIGH);  // step if the number of interrupts cycles is greater than a threshold
		checkHitEndstops_z();
		WRITE(Z_STEP_P,LOW);  // after 1us, pull the pin low and increment the step counter
		for (unsigned int j=0; j<(unsigned int)dc; j++) {
			__asm__("nop\n\t");
		}
		steps++;
		reading = sma_filter(analogRead(LC_ANALOG_PIN));
		dc = 0.9772*dc;
	}
	zero_reading = reading;
	reading = 0;

	while ((abs(reading) < threshold) && (steps<(unsigned long)(4.0*AXIS_STEPS_PER_UNIT_Z))) {
		//Constant speed phase

		WRITE(Z_STEP_P, HIGH);  // step if the number of interrupts cycles is greater than a threshold
		checkHitEndstops_z();
		WRITE(Z_STEP_P,LOW);  // after 1us, pull the pin low and increment the step counter
		for (unsigned int j=0; j<10; j++) {
			__asm__("nop\n\t");
		}
		steps++;
		reading = sma_filter(analogRead(LC_ANALOG_PIN))- zero_reading;
	}

	sei();

    WRITE(ENABLE_ANALOG_PIN, HIGH);

	if (motionz.touchswith_z_max == true) {
		motionz.pos_z = MAX_DIST_Z;
	}
	else if (motionz.touchswith_z_min == true) {
		motionz.pos_z = 0.0;
	}
	else {
		motionz.pos_z = motionz.pos_z + (float)steps*1.0/AXIS_STEPS_PER_UNIT_Z;
	}

	Z_init();

	return motionz.pos_z;

}


float find_surface_cont() {

	unsigned long steps=0;

	bool readout = false;

	Z_init();  // the homing function has already initialized the variables and activated motionz.homing

	motionz.direction_z = 1;

	set_directions_z();  // pull the pins depending on the directions

	enable_z();
	
	delay(100);



	cli(); //disable interrupts

	float dc = 1000;

	while ((unsigned int)dc>100) {
		WRITE(Z_STEP_P, HIGH);  // step if the number of interrupts cycles is greater than a threshold
		checkHitEndstops_z();
		WRITE(Z_STEP_P,LOW);  // after 1us, pull the pin low and increment the step counter
		for (unsigned int j=0; j<(unsigned int)dc; j++) {
			__asm__("nop\n\t");
		}
		steps++;
		dc = 0.9772*dc;
	}

	while ((readout==false) && (steps<(unsigned long)(5.0*AXIS_STEPS_PER_UNIT_Z))) {
		//Constant speed phase

		WRITE(Z_STEP_P, HIGH);  // step if the number of interrupts cycles is greater than a threshold
		checkHitEndstops_z();
		WRITE(Z_STEP_P,LOW);  // after 1us, pull the pin low and increment the step counter
		for (unsigned int j=0; j<100; j++) {
			__asm__("nop\n\t");
		}
		readout = READ(CONTACT_PIN);
		steps++;
	}

	sei();

	if (motionz.touchswith_z_max == true) {
		motionz.pos_z = MAX_DIST_Z;
	}
	else if (motionz.touchswith_z_min == true) {
		motionz.pos_z = 0.0;
	}
	else {
		motionz.pos_z = motionz.pos_z + (float)steps*1.0/AXIS_STEPS_PER_UNIT_Z;
	}

	Z_init();

	return motionz.pos_z;

}