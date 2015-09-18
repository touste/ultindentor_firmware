// SerialRefs.h

#ifndef _SERIALREFS_h
#define _SERIALREFS_h


void Serial_init();

void unrecognized(const char *command);

void moveXY_rel_serial();
void moveXY_abs_serial();
void moveZ_rel_serial();
void moveZ_abs_serial();
void SelectProbe_serial();
void home_XY_serial(); 
void XY_init_absolute_serial();
void home_Z_serial(); 
void Z_init_absolute_serial(); 
void find_surface_serial();
void get_pos_serial();
void shutdown_serial();

#endif

