#ifndef XYMOTION_H
#define XYMOTION_H


typedef struct{

  unsigned long rel_steps_x;  // total steps required to achieve the motion
  unsigned long rel_steps_y; 

  float pos_x;  // absolution positions in mm
  float pos_y;

  unsigned long step_accel_xy;

  unsigned long step_accel_x;
  unsigned long step_descel_x;
  unsigned long step_constant_x;

  unsigned long step_accel_y;
  unsigned long step_descel_y;
  unsigned long step_constant_y;

  int direction_x;  // -1 is backward, 0 means no motion, 1 is forward
  int direction_y;

  float nominal_speed_xy;   // target speed in mm/s

  float last_speed_xy;   // last known speed
  unsigned int last_step_accel_xy;

  bool touchswith_x_min;  
  bool touchswith_y_min;  
  bool touchswith_x_max;  
  bool touchswith_y_max;  
  
} motion_xy;  // structure containing all you need to know about a motion


void home_XY();  // homing function

void moveXY_abs(float x, float y, float speed);  // absolute motion, relies on the relative motion function

void XY_init();  // initialize the motion variables

void X_init();  // initialize the motion variables

void Y_init();  // initialize the motion variables

void XY_init_absolute();  // initialize the motion variables and the initial positions after homing

void moveXY_rel(float x, float y, float speed);  // move relative to the current position; calculate all the motion variables and start the interrupt

void init_endstops_xy();  // initialize endstops pins

void set_directions_xy();  // set the direction pins for the motion


#endif
