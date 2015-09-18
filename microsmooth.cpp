/*
Microsmooth, DSP library for Arduino
Copyright (C) 2013, Asheesh Ranjan, Pranav Jetley

This program is free software; you can redistribute it and/or modify it under the terms of the GNU General Public License as published by the Free Software Foundation; either version 2 of the License, or (at your option) any later version.

This program is distributed in the hope that it will be useful, but WITHOUT ANY WARRANTY; without even the implied warranty of MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE. See the GNU General Public License for more details.

You should have received a copy of the GNU General Public License along with this program; if not, write to the Free Software Foundation, Inc., 51 Franklin Street, Fifth Floor, Boston, MA 02110-1301 USA.
*/


#include "microsmooth.h"
#include <Arduino.h>

unsigned int res_array[SMA_LENGTH];

void init_sma(unsigned int current_value) {
	unsigned int i;
	for(i=0;i<SMA_LENGTH;i++)
    {
	res_array[i]=current_value;
    }
}
unsigned int sma_filter(unsigned int current_value)
{  
    unsigned long sum=0; /*This constrains SMA_LENGTH*/
    unsigned int average=0;
    unsigned int i;

    for(i=1;i<SMA_LENGTH;i++)
    {
	res_array[i-1]=res_array[i];
    }
    res_array[SMA_LENGTH-1]=current_value;
    
    for(i=0;i<SMA_LENGTH;i++)
    {
	sum+=(unsigned long)res_array[i];
    }
    average=(unsigned int)(sum/SMA_LENGTH);

    return average;
}
