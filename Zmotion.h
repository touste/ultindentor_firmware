#ifndef ZMOTION_H
#define ZMOTION_H


typedef struct{


  unsigned long rel_steps_z;  // total steps required to achieve the motion

  float pos_z;  // absolution positions in mm

  unsigned long step_accel_z;
  unsigned long step_descel_z;
  unsigned long step_constant_z;

  int direction_z;  // -1 is backward, 0 means no motion, 1 is forward

  float nominal_speed_z;   // target speed in mm/s

  float last_speed_z;   // last known speed
  unsigned int last_step_accel_z;

  bool last_hardstop_z;   // last known speed
  unsigned int last_step_descel_hardstop_z;
  
  bool touchswith_z_min; 
  bool touchswith_z_max; 

} motion_z;  // structure containing all you need to know about a motion


void home_Z();  // homing function

void moveZ_abs(float z, float speed, bool hardstop = false, bool get_contact = false, float indent_after_contact = 1.0);  // absolute motion, relies on the relative motion function

void Z_init();  // initialize the motion variables

void Z_init_absolute();  // initialize the motion variables and the initial positions after homing

void moveZ_rel(float z, float speed, bool hardstop = false, bool get_contact = false, float indent_after_contact = 1.0);  // move relative to the current position; calculate all the motion variables and start the motion

void init_endstops_z();  // initialize endstops pins

void set_directions_z();  // set the direction pins for the motion

float find_surface_lc(int threshold);

float find_surface_cont();


#endif
