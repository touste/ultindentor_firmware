



#include "defs.h"


extern motion_xy motionxy;
extern motion_z motionz;
extern SerialCommand SCmd;




void setup_poweron()
{
    // Power supply ON
   SET_OUTPUT(SUICIDE_PIN);
   WRITE(SUICIDE_PIN, HIGH);
   SET_OUTPUT(PS_ON_PIN);
   WRITE(PS_ON_PIN, HIGH);
   	SET_OUTPUT(O19V_ENABLE_PIN);
	WRITE(O19V_ENABLE_PIN, HIGH);
}


void setup_poweroff()
{
    // Power supply ON
   SET_OUTPUT(SUICIDE_PIN);
   WRITE(SUICIDE_PIN, LOW);
   SET_OUTPUT(PS_ON_PIN);
   WRITE(PS_ON_PIN, LOW);
}

void setup_axes()
{
	SET_OUTPUT(Z_ENABLE_P);
	WRITE(Z_ENABLE_P, HIGH);
	SET_OUTPUT(Z_ENABLE_GND);
	WRITE(Z_ENABLE_GND, LOW);

	SET_OUTPUT(Z_STEP_P);
	WRITE(Z_STEP_P, LOW);
	SET_OUTPUT(Z_STEP_GND);
	WRITE(Z_STEP_GND, LOW);

	SET_OUTPUT(Z_DIR_P);
	WRITE(Z_DIR_P, LOW);
	SET_OUTPUT(Z_DIR_GND);
	WRITE(Z_DIR_GND, LOW);

	SET_OUTPUT(X_ENABLE_PIN);
	WRITE(X_ENABLE_PIN, LOW);

	SET_OUTPUT(X_STEP_PIN);
	WRITE(X_STEP_PIN, LOW);

	SET_OUTPUT(X_DIR_PIN);
	WRITE(X_DIR_PIN, LOW);

	SET_OUTPUT(Y_ENABLE_PIN);
	WRITE(Y_ENABLE_PIN, LOW);

	SET_OUTPUT(Y_STEP_PIN);
	WRITE(Y_STEP_PIN, LOW);

	SET_OUTPUT(Y_DIR_PIN);
	WRITE(Y_DIR_PIN, LOW);

	motionz.last_speed_z = 0;
	motionxy.last_speed_xy = 0;

	motionz.last_hardstop_z = false;
}

void setup_OUTs()
{
	SET_OUTPUT(ENABLE_LC1_PIN);
	SET_OUTPUT(ENABLE_LC2_PIN);
	SET_OUTPUT(ENABLE_ANALOG_PIN);
	SET_OUTPUT(ENABLE_SERVO_PIN);
	WRITE(ENABLE_LC1_PIN, HIGH);
	WRITE(ENABLE_LC2_PIN, HIGH);
	WRITE(ENABLE_ANALOG_PIN, LOW);
	WRITE(ENABLE_SERVO_PIN, HIGH);

	SET_OUTPUT(START_ACQ_PIN);
	WRITE(START_ACQ_PIN, LOW);

	SET_INPUT(LC_ANALOG_PIN);
	SET_INPUT(CONTACT_PIN);
}



void setup() {
  Serial.begin(115200);
  setup_poweron();
  setup_axes();
  setup_OUTs();
  home_Z();
  home_XY();
  SelectProbe(1);
  delay(100);
  Serial_init();
  Serial.print("READY!\n");
}

void loop() {
	SCmd.readSerial();
}