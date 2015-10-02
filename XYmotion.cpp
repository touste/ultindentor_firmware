// 
// 
// 

#include "XYmotion.h"
#include "defs.h"
#include "gen_tcounts_curves.h"

unsigned int curve_table_xy[800];
motion_xy motionxy;
extern SerialCommand SCmd;



FORCE_INLINE void quickStop_x()  // stop now! and reinitialize everything
{
	disable_x();
	X_init();
}

FORCE_INLINE void quickStop_y()  // stop now! and reinitialize everything
{
	disable_y();
	Y_init();
}


FORCE_INLINE void checkHitEndstops_x()  // check if the endstops are activated. if homing, do not completely stop the motion (stop only the concerned axis)
{
	if ((motionxy.direction_x==1) && (READ(X_MIN_PIN) != X_MIN_ENDSTOP_INVERTING))
	{
		quickStop_x();
		#ifdef DEBUG
			sei();
			Serial.print("X_min endswitch hit!\n\n");
		#endif
		motionxy.touchswith_x_min = true;
		return;
	}
	else if ((motionxy.direction_x==-1) && (READ(X_MAX_PIN) != X_MAX_ENDSTOP_INVERTING))
	{
		quickStop_x();
		#ifdef DEBUG
			sei();
			Serial.print("X_max endswitch hit!\n\n");
		#endif
		motionxy.touchswith_x_max = true;
		return;
	}

}

FORCE_INLINE void checkHitEndstops_y()  // check if the endstops are activated. if homing, do not completely stop the motion (stop only the concerned axis)
{
	if ((motionxy.direction_y==1) && (READ(Y_MIN_PIN) != Y_MIN_ENDSTOP_INVERTING))
	{
		quickStop_y();
		#ifdef DEBUG
			sei();
			Serial.print("Y_min endswitch hit!\n\n");
		#endif
		motionxy.touchswith_y_min = true;
		return;
	}
	else if ((motionxy.direction_y==-1) && (READ(Y_MAX_PIN) != Y_MAX_ENDSTOP_INVERTING))
	{
		quickStop_y();
		#ifdef DEBUG
			sei();
			Serial.print("Y_max endswitch hit!\n\n");
		#endif
		motionxy.touchswith_y_max = true;
		return;
	}

}



void home_XY()
{
	moveXY_rel(-500, -500, HOMING_SPEED_XY);  // arbitrary values, just to be sure home is reached
	XY_init_absolute();  // initialize everything
}


void moveXY_abs(float x, float y, float speed)
{
	moveXY_rel(x - motionxy.pos_x, y - motionxy.pos_y, speed);  //  transform an absolute motion to a relative one
}



void X_init()  //  initialize all motion variables
{

  motionxy.rel_steps_x = 0;

  motionxy.step_accel_x = 0;
  motionxy.step_descel_x = 0;
  motionxy.step_constant_x = 0;

  motionxy.direction_x = 0;

  motionxy.touchswith_x_min = false;
  motionxy.touchswith_x_max = false;

  disable_x();
}


void Y_init()  //  initialize all motion variables
{

  motionxy.rel_steps_y = 0;

  motionxy.step_accel_y = 0;
  motionxy.step_descel_y = 0;
  motionxy.step_constant_y = 0;

  motionxy.direction_y = 0;

  motionxy.touchswith_y_min = false;
  motionxy.touchswith_y_max = false;

  disable_y();
}


void XY_init()
{

  X_init();
  Y_init();

  motionxy.step_accel_xy = 0;
  motionxy.nominal_speed_xy = 0;

}


void XY_init_absolute()  // same as above, nut with the absolute positions as well
{

  XY_init();

  motionxy.pos_x = 0;
  motionxy.pos_y = 0;


}


void moveXY_rel(float x, float y, float speed)  // we need to fill all these motion variables!
{

	

	XY_init();


	// the following pre-defines the directions based on the signs of x and y
	if (fabs(x)<MIN_MOTION_XY)
	{
		motionxy.direction_x = 0;
		x = 0;
	}
	else if (x>0) 
	{
		motionxy.direction_x = 1;
	}
	else if (x<0)
	{
		motionxy.direction_x = -1;
	}


	if (fabs(y)<MIN_MOTION_XY)
	{
		motionxy.direction_y = 0;
		y = 0;
	}
	else if (y>0) 
	{
		motionxy.direction_y = 1;
	}
	else if (y<0)
	{
		motionxy.direction_y = -1;
	}

	set_directions_xy();  // pull the pins depending on the directions


	speed = fabs(speed);
	speed = constrain(speed, MIN_SPEED_XY, MAX_SPEED_XY);  // no more, no less

	if (speed != motionxy.last_speed_xy) {
				motionxy.step_accel_xy = (unsigned long)gen_timercounts(curve_table_xy, speed, INTERRUPT_FREQ_XY, AXIS_STEPS_PER_UNIT_XY, JERK_XY, ACCEL_XY, CPUCYCLES_XY, SCURVES_XY); //The calculation of the speed curve is done now (it takes some time!)
			}
	else
			{
				motionxy.step_accel_xy = motionxy.last_step_accel_xy;
			}
	
	motionxy.last_speed_xy = speed;
	motionxy.last_step_accel_xy = motionxy.step_accel_xy;

	enable_x();
	enable_y();

	// the following calculates the steps. special treatment is needed when x or y is zero
	if (x==0)  
	{

		if (y==0)
		{
			XY_init();
			sei();
			return; // no motion
		}
		else  // x is zero, y is not
		{
			X_init();
		}

	}
	else if (y==0)  // same as previously, but his time y is zero
	{
			Y_init();
	}


	motionxy.rel_steps_y = (unsigned long) (fabs(y) * (float)AXIS_STEPS_PER_UNIT_XY);	
	motionxy.step_accel_y = motionxy.step_accel_xy;

	if (motionxy.step_accel_y >= (unsigned long) ((float)motionxy.rel_steps_y/2.0)) {
		motionxy.step_accel_y = (unsigned long) ((float)motionxy.rel_steps_y/2.0);
	}

	motionxy.step_descel_y = motionxy.step_accel_y;
	motionxy.step_constant_y = motionxy.rel_steps_y - motionxy.step_accel_y - motionxy.step_descel_y;


	motionxy.rel_steps_x = (unsigned long) (fabs(x) * (float)AXIS_STEPS_PER_UNIT_XY);	
	motionxy.step_accel_x = motionxy.step_accel_xy;

	if (motionxy.step_accel_x >= (unsigned long) ((float)motionxy.rel_steps_x/2.0)) {
		motionxy.step_accel_x = (unsigned long) ((float)motionxy.rel_steps_x/2.0);
	}

	motionxy.step_descel_x = motionxy.step_accel_x;
	motionxy.step_constant_x = motionxy.rel_steps_x - motionxy.step_accel_x - motionxy.step_descel_x;


	init_endstops_xy();

	float dcount_min = 1.0/(float)AXIS_STEPS_PER_UNIT_XY*(float)INTERRUPT_FREQ_XY/speed;
	unsigned int constantspeed_dc_x = (unsigned int) dcount_min - (unsigned int) CPUCYCLES_XY;
	unsigned int constantspeed_dc_y = (unsigned int) dcount_min - (unsigned int) CPUCYCLES_XY;

	if ((constantspeed_dc_x<5) || (constantspeed_dc_y<5)) { // I don't know what went wrong but this is bad
		XY_init();
		sei();
		return;
	}
	
	// Start the X motion:
	#ifdef DEBUG
		Serial.print("Initial absolute x position: ");
		Serial.print(motionxy.pos_x);
		Serial.print("\nThe X motion is starting:\nRelative motion of ");
		Serial.print(x);
		Serial.print(" mm at a speed of ");
		Serial.print(speed);
		Serial.print("\nThis amounts to:\nTotal number of steps: ");
		Serial.print(motionxy.rel_steps_x);
		Serial.print(".\nAcceleration steps: ");
		Serial.print(motionxy.step_accel_x);
		Serial.print(".\nConstant speed steps: ");
		Serial.print(motionxy.step_constant_x);
		Serial.print(".\nDesceleration steps: ");
		Serial.print(motionxy.step_descel_x);
		Serial.print(".\nConstant speed dc: ");
		Serial.print(constantspeed_dc_x);
		Serial.print(".\n\n");
	#endif


	cli(); //disable interrupts


	//Acceleration phase
	for (unsigned long i=0; i<motionxy.step_accel_x; i++) {
			WRITE(X_STEP_PIN, HIGH);  // step if the number of interrupts cycles is greater than a threshold
			checkHitEndstops_x();
			WRITE(X_STEP_PIN,LOW);  // after 1us, pull the pin low and increment the step counter
			for (unsigned int j=0; j<curve_table_xy[i]; j++) {
				__asm__("nop\n\t");
			}
	}


	//Constant speed phase
	for (unsigned long i=0; i<motionxy.step_constant_x; i++) {
			WRITE(X_STEP_PIN, HIGH);  // step if the number of interrupts cycles is greater than a threshold
			checkHitEndstops_x();
			WRITE(X_STEP_PIN,LOW);  // after 1us, pull the pin low and increment the step counter
			for (unsigned int j=0; j<constantspeed_dc_x; j++) {
				__asm__("nop\n\t");
			}
	}

		
	//Desceleration phase
	for (unsigned long i=motionxy.step_descel_x; i>0; i--) {
			WRITE(X_STEP_PIN, HIGH);  // step if the number of interrupts cycles is greater than a threshold
			checkHitEndstops_x();
			WRITE(X_STEP_PIN,LOW);  // after 1us, pull the pin low and increment the step counter
			for (unsigned int j=0; j<curve_table_xy[i-1]; j++) {
				__asm__("nop\n\t");
			}
	}

	sei(); //reenable interrupts

	if (motionxy.touchswith_x_min == true) {
		motionxy.pos_x = MAX_DIST_X;
	}
	else if (motionxy.touchswith_x_max == true) {
		motionxy.pos_x = 0.0;
	}
	else {
		motionxy.pos_x = motionxy.pos_x + x;
	}

	#ifdef DEBUG
		Serial.print("Final absolute x position: ");
		Serial.print(motionxy.pos_x);
		Serial.print(".\n\n");
	#endif

	// Start the Y motion:

	#ifdef DEBUG
		Serial.print("Initial absolute y position: ");
		Serial.print(motionxy.pos_y);
		Serial.print("\nThe Y motion is starting:\nRelative motion of ");
		Serial.print(y);
		Serial.print(" mm at a speed of ");
		Serial.print(speed);
		Serial.print("\nThis amounts to:\nTotal number of steps: ");
		Serial.print(motionxy.rel_steps_y);
		Serial.print(".\nAcceleration steps: ");
		Serial.print(motionxy.step_accel_y);
		Serial.print(".\nConstant speed steps: ");
		Serial.print(motionxy.step_constant_y);
		Serial.print(".\nDesceleration steps: ");
		Serial.print(motionxy.step_descel_y);
		Serial.print(".\nConstant speed dc: ");
		Serial.print(constantspeed_dc_y);
		Serial.print(".\n\n");
	#endif

	cli(); //disable interrupts

	//Acceleration phase
	for (unsigned long i=0; i<motionxy.step_accel_y; i++) {
			WRITE(Y_STEP_PIN, HIGH);  // step if the number of interrupts cycles is greater than a threshold
			checkHitEndstops_y();
			WRITE(Y_STEP_PIN,LOW);  // after 1us, pull the pin low and increment the step counter
			for (unsigned int j=0; j<curve_table_xy[i]; j++) {
				__asm__("nop\n\t");
			}
	}


	//Constant speed phase
	for (unsigned long i=0; i<motionxy.step_constant_y; i++) {
			WRITE(Y_STEP_PIN, HIGH);  // step if the number of interrupts cycles is greater than a threshold
			checkHitEndstops_y();
			WRITE(Y_STEP_PIN,LOW);  // after 1us, pull the pin low and increment the step counter
			for (unsigned int j=0; j<constantspeed_dc_y; j++) {
				__asm__("nop\n\t");
			}
	}

		
	//Desceleration phase
	for (unsigned long i=motionxy.step_descel_y; i>0; i--) {
			WRITE(Y_STEP_PIN, HIGH);  // step if the number of interrupts cycles is greater than a threshold
			checkHitEndstops_y();
			WRITE(Y_STEP_PIN,LOW);  // after 1us, pull the pin low and increment the step counter
			for (unsigned int j=0; j<curve_table_xy[i-1]; j++) {
				__asm__("nop\n\t");
			}
	}

	sei(); //reenable interrupts

	if (motionxy.touchswith_y_min == true) {
		motionxy.pos_y = MAX_DIST_Y;
	}
	else if (motionxy.touchswith_y_max == true) {
		motionxy.pos_y = 0.0;
	}
	else {
		motionxy.pos_y = motionxy.pos_y + y;
	}

	#ifdef DEBUG
		Serial.print("Final absolute y position: ");
		Serial.print(motionxy.pos_y);
		Serial.print(".\n\n");
	#endif

	XY_init();

	

}


void set_directions_xy()  // pull pins depending on the directions of the motion
{
	SET_OUTPUT(X_DIR_PIN);
	
    SET_OUTPUT(Y_DIR_PIN);


	if (motionxy.direction_x==1) 
	{
		WRITE(X_DIR_PIN, FORWARD_X);
	}
	else if (motionxy.direction_x==-1)
	{
		WRITE(X_DIR_PIN, !FORWARD_X);
	}

	if (motionxy.direction_y==1) 
	{
		WRITE(Y_DIR_PIN, FORWARD_Y);
	}
	else if (motionxy.direction_y==-1)
	{
		WRITE(Y_DIR_PIN, !FORWARD_Y);
	}

}

void init_endstops_xy()
{
	//endstops and pullups

    SET_INPUT(X_MIN_PIN);
    WRITE(X_MIN_PIN,HIGH);

    SET_INPUT(Y_MIN_PIN);
    WRITE(Y_MIN_PIN,HIGH);

    SET_INPUT(X_MAX_PIN);
    WRITE(X_MAX_PIN,HIGH);

    SET_INPUT(Y_MAX_PIN);
    WRITE(Y_MAX_PIN,HIGH);

}
