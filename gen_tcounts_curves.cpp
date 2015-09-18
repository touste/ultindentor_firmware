// 
// 
// 

#include "gen_tcounts_curves.h"
#include "defs.h"

unsigned int gen_timercounts(unsigned int *curve_table, float target_speed, float freq, float steps_per_unit, float jerk, float acc, unsigned long cpucycles, bool scurve)
{


	if (scurve == true) {

		Serial.print("Calculating new S_curve table for new speed (can take a while)...\n\n");


		float dcount_min=1.0/(float)steps_per_unit*(float)freq/target_speed;
		unsigned long dcount = constrain((unsigned long)dcount_min/10,1,50);

		float cycles_half = sqrt((target_speed)/(float)jerk)*(float)freq;
		float j;
		float int_jerk = 0;
		float acc;
		float int_acc = 0;
		float int_speed = 0;
		float speed;
		float pos;
		unsigned int n_step = 0;
		unsigned long temptable;

		float dtime=(float)dcount/(float)freq;
		float dx=1.0/(float)steps_per_unit;


		unsigned long prec_step = 0;

		j = (float)jerk;
		for (unsigned long i=1; i<=((unsigned long)cycles_half); i=i+dcount) {
			int_jerk=int_jerk+j;
			acc=int_jerk*dtime;
			int_acc=int_acc+acc;
			speed=int_acc*dtime;
			int_speed=int_speed+speed;
			pos=int_speed*dtime;

			if (pos>(((float)n_step+1.0)*dx)) {
				n_step++;
				temptable = constrain(i-prec_step-(unsigned long)cpucycles,0,65535);
				curve_table[n_step-1]= (unsigned int) temptable;
				prec_step = i;
				//#ifdef DEBUG
				//	Serial.print("n_step: ");
				//	Serial.print(n_step);
				//	Serial.print("    cpu_count: ");
				//	Serial.print(temptable);
				//	Serial.print(".\n");
				//#endif
			}
		}


		j = -(float)jerk;
		for (unsigned long i=(unsigned long)cycles_half+1; i<=(2*(unsigned long)cycles_half); i=i+dcount) {
			int_jerk=int_jerk+j;
			acc=int_jerk*dtime;
			int_acc=int_acc+acc;
			speed=int_acc*dtime;
			int_speed=int_speed+speed;
			pos=int_speed*dtime;

			if (pos>(((float)n_step+1.0)*dx)) {
				n_step++;
				temptable = constrain(i-prec_step-(unsigned long)cpucycles,0,65535);
				curve_table[n_step-1]= (unsigned int) temptable;
				prec_step = i;
				//#ifdef DEBUG
				//	Serial.print("n_step: ");
				//	Serial.print(n_step);
				//	Serial.print("    cpu_count: ");
				//	Serial.print(temptable);
				//	Serial.print(".\n");
				//#endif
			}
		}
		

		if (n_step == 0) {
			n_step = 1;
			curve_table[0] = constrain((unsigned int) dcount_min,5,65535);
		}

		Serial.print("Done!\n\n");
		return n_step;
	}
	else {

		Serial.print("Calculating new table for new speed (can take a while)...\n\n");

		float dcount_min=1.0/(float)steps_per_unit*(float)freq/target_speed;
		unsigned long dcount = constrain((unsigned long)dcount_min/10,1,50);

		float cycles = target_speed/(float)acc*(float)freq;
		float int_acc = 0;
		float int_speed = 0;
		float speed;
		float pos;
		unsigned int n_step = 0;
		unsigned long temptable;

		float dtime=(float)dcount/(float)freq;
		float dx=1.0/(float)steps_per_unit;


		unsigned long prec_step = 0;

		for (unsigned long i=1; i<=((unsigned long)cycles); i=i+dcount) {
			int_acc=int_acc+(float)acc;
			speed=int_acc*dtime;
			int_speed=int_speed+speed;
			pos=int_speed*dtime;

			if (pos>(((float)n_step+1.0)*dx)) {
				n_step++;
				temptable = constrain(i-prec_step-(unsigned long)cpucycles,0,65535);
				curve_table[n_step-1]= (unsigned int) temptable;
				prec_step = i;
				//#ifdef DEBUG
				//	Serial.print("n_step: ");
				//	Serial.print(n_step);
				//	Serial.print("    cpu_count: ");
				//	Serial.print(temptable);
				//	Serial.print(".\n");
				//#endif
			}
		}

		if (n_step == 0) {
			n_step = 1;
			curve_table[0] = constrain((unsigned int) dcount_min,5,65535);
		}


		Serial.print("Done!\n\n");
		return n_step;
	}
}