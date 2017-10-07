#include "PID.h"

float tempError[3],y_output[2];
float update_PID(float error)
{
	tempError[0] = tempError[1];
	tempError[1] = tempError[2];
	tempError[2] = error;
	
	y_output[0] = y_output[1];
	y_output[1] = y_output[0] + 
	a * tempError[2] + b * tempError[1] + c * tempError[0];
	return y_output[1];
}
// Please Calculate a,b,c befor running the PID function
