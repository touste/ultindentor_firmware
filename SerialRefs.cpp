// 
// 
// 

#include "SerialRefs.h"
#include "defs.h"


SerialCommand SCmd;
extern motion_xy motionxy;
extern motion_z motionz;

void Serial_init()
{
SCmd.addCommand("MOVRELXY", moveXY_rel_serial);    
SCmd.addCommand("MOVABSXY", moveXY_abs_serial); 
SCmd.addCommand("MOVRELZ", moveZ_rel_serial);    
SCmd.addCommand("MOVABSZ", moveZ_abs_serial); 
SCmd.addCommand("HOMEXY", home_XY_serial); 
SCmd.addCommand("RESET_POSXY", XY_init_absolute_serial);
SCmd.addCommand("HOMEZ", home_Z_serial); 
SCmd.addCommand("RESET_POSZ", Z_init_absolute_serial); 
SCmd.addCommand("SELHEAD", SelectProbe_serial);
SCmd.addCommand("FINDSURFACE", find_surface_serial);
SCmd.addCommand("GETPOS", get_pos_serial);
SCmd.addCommand("SHUTDOWN", shutdown_serial);
SCmd.setDefaultHandler(unrecognized);
}


// This gets set as the default handler, and gets called when no other command matches.
void unrecognized(const char *command) {
  Serial.println("What?");
}

void moveXY_rel_serial()
{

  float posx = 0.0;
  float posy = 0.0;
  float speed = 0.0;  
  char *arg; 
  
  arg = SCmd.next(); 
  if (arg != NULL) 
  {
    posx=atof(arg);    // Converts a char string to float
  } 
  else return;
 
  arg = SCmd.next(); 
  if (arg != NULL) 
  {
    posy=atof(arg);    // Converts a char string to float
  } 
  else return;

  arg = SCmd.next(); 
  if (arg != NULL) 
  {
    speed=atof(arg);    // Converts a char string to float
  } 
  else return;

  Serial.print("Relative XY motion to (");
  Serial.print(posx);
  Serial.print(", ");
  Serial.print(posy);
  Serial.print(") at a speed of ");
  Serial.print(speed);
  Serial.print(".\n");
  moveXY_rel(posx, posy, speed);
  Serial.print("MOVRELXY DONE\n\n");
}


void moveXY_abs_serial()
{

  float posx = 0.0;
  float posy = 0.0;
  float speed = 0.0;  
  char *arg; 

  arg = SCmd.next(); 
  if (arg != NULL) 
  {
    posx=atof(arg);    // Converts a char string to an integer
  } 
  else return;
 
  arg = SCmd.next(); 
  if (arg != NULL) 
  {
    posy=atof(arg);    // Converts a char string to an integer
  } 
  else return;

  arg = SCmd.next(); 
  if (arg != NULL) 
  {
    speed=atof(arg);    // Converts a char string to an integer
  } 
  else return;

  Serial.print("Absolute XY motion to (");
  Serial.print(posx);
  Serial.print(", ");
  Serial.print(posy);
  Serial.print(") at a speed of ");
  Serial.print(speed);
  Serial.print(".\n");

  moveXY_abs(posx, posy, speed);
  Serial.print("MOVABSXY DONE\n\n");
}



void moveZ_rel_serial()
{

  float posz = 0.0;
  float speed = 0.0;  
  int hardstop = 0;
  int contact = 0;
  float indent = 1.0;
  char *arg; 

  arg = SCmd.next(); 
  if (arg != NULL) 
  {
    posz=atof(arg);    // Converts a char string to an integer
  }
  else return;
 
  arg = SCmd.next(); 
  if (arg != NULL) 
  {
    speed=atof(arg);    // Converts a char string to an integer
  } 
  else return;

  arg = SCmd.next(); 
  if (arg != NULL) 
  {
    hardstop=atoi(arg);    // Converts a char string to an integer
  } 

  arg = SCmd.next(); 
  if (arg != NULL) 
  {
    contact=atoi(arg);    // Converts a char string to an integer
  }

  arg = SCmd.next(); 
  if (arg != NULL) 
  {
    indent=atof(arg);    // Converts a char string to an integer
  }

  Serial.print("Relative Z motion to ");
  Serial.print(posz);
  Serial.print(" at a speed of ");
  Serial.print(speed);
  Serial.print(".\n");
  moveZ_rel(posz, speed, (hardstop != 0), (contact != 0), indent);
  Serial.print("MOVRELZ DONE\n\n");
}


void moveZ_abs_serial()
{

  float posz = 0.0;
  float speed = 0.0;  
  int hardstop = 0;
  int contact = 0;
  float indent = 1.0;
  char *arg; 

  arg = SCmd.next(); 
  if (arg != NULL) 
  {
    posz=atof(arg);    // Converts a char string to an integer
  } 
  else return;

  arg = SCmd.next(); 
  if (arg != NULL) 
  {
    speed=atof(arg);    // Converts a char string to an integer
  } 
  else return;

  arg = SCmd.next(); 
  if (arg != NULL) 
  {
    hardstop=atoi(arg);    // Converts a char string to an integer
  } 

  arg = SCmd.next(); 
  if (arg != NULL) 
  {
    contact=atoi(arg);    // Converts a char string to an integer
  }

  arg = SCmd.next(); 
  if (arg != NULL) 
  {
    indent=atof(arg);    // Converts a char string to an integer
  }

  Serial.print("Absolute Z motion to ");
  Serial.print(posz);
  Serial.print(" at a speed of ");
  Serial.print(speed);
  Serial.print(".\n");

  moveZ_abs(posz, speed, (hardstop != 0), (contact != 0), indent);
  Serial.print("MOVABSZ DONE\n\n");
}



void SelectProbe_serial()
{

  int probe = 0;
  char *arg; 

  arg = SCmd.next(); 
  if (arg != NULL) 
  {
    probe=atoi(arg);    // Converts a char string to an integer
  } 
  else return;

  Serial.print("Selecting probe number ");
  Serial.print(probe);
  Serial.print(".\n");

  SelectProbe(probe);
  Serial.print("SELHEAD DONE\n\n");
}



void home_XY_serial() {
  Serial.print("Homing XY axes.\n");
  home_XY();
  Serial.print("HOMEXY DONE\n\n");
}

void XY_init_absolute_serial() {
  Serial.print("Resetting XY position.\n");
  XY_init_absolute();
  Serial.print("RESET_POSXY DONE\n\n");
}

void home_Z_serial() {
  Serial.print("Homing Z axis.\n");
  home_Z();
  Serial.print("HOMEZ DONE\n\n");
}

void Z_init_absolute_serial() {
  Serial.print("Resetting Z position.\n");
  Z_init_absolute();
  Serial.print("RESET_POSZ DONE\n\n");
}





void find_surface_serial()
{
  float pos = 0.0;
  int threshold = 0;
  char *arg; 

  arg = SCmd.next(); 
  if (arg != NULL) 
  {
    threshold=atoi(arg);    // Converts a char string to an integer
  } 
  else return;

  Serial.print("Finding material surface...\n\n");


  //pos=find_surface_lc(threshold);
  pos=find_surface_cont();

  Serial.print("POS: ");
  Serial.print(pos);
  Serial.print("\n");

  Serial.print("FINDSURFACE DONE\n\n");
}


void get_pos_serial() {
	char *arg; 
	int numaxis = 0;

	arg = SCmd.next(); 
	if (arg != NULL) 
	{
	numaxis=atoi(arg);    // Converts a char string to an integer
	} 
	else return;

	delay(200);

	Serial.print("POS: ");

	switch(numaxis) {
	case 1: Serial.print(motionxy.pos_x);
		    Serial.print("\n");
			break;
	case 2: Serial.print(motionxy.pos_y);
		    Serial.print("\n");
			break;
	case 3: Serial.print(motionz.pos_z);
		    Serial.print("\n");
			break;
	}

	Serial.print("GETPOS DONE\n\n");
}


void shutdown_serial() {
	
	WRITE(SUICIDE_PIN, LOW);
    WRITE(PS_ON_PIN, LOW);
	WRITE(O19V_ENABLE_PIN, LOW);


	Serial.print("SHUTDOWN DONE\n\n");
}