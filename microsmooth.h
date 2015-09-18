/*
Microsmooth, DSP library for Arduino
Copyright (C) 2013, Asheesh Ranjan, Pranav Jetley

This program is free software; you can redistribute it and/or modify it under the terms of the GNU General Public License as published by the Free Software Foundation; either version 2 of the License, or (at your option) any later version.

This program is distributed in the hope that it will be useful, but WITHOUT ANY WARRANTY; without even the implied warranty of MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE. See the GNU General Public License for more details.

You should have received a copy of the GNU General Public License along with this program; if not, write to the Free Software Foundation, Inc., 51 Franklin Street, Fifth Floor, Boston, MA 02110-1301 USA.
*/

/*Standard Library Dependencies*/
#include <stdint.h>
//#include <math.h>
#include <stdlib.h>

#ifndef MICROSMOOTH
#define MICROSMOOTH

/*Algorithm Parameters*/

/*
These parameters should be tuned depending on need. Each of these parameters affects
run time and signal smoothing obtained. See documentation for specific instructions 
on tuning these parameters.
*/

/*Simple Moving Average - Length of window */
#ifndef SMA_LENGTH
#define SMA_LENGTH 100
#endif

/*Function Prototypes*/
void init_sma(unsigned int current_value);
unsigned int sma_filter(unsigned int current_value);

#endif

