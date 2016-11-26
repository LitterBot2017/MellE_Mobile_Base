/*
 * PID.cpp
 *
 *  Created on: Nov 9, 2016
 *      Author: Admin
 */

#include "PID_horz.h"
#include "stdlib.h"

using namespace std;

PID_horz::PID_horz(float d, float p, float i, float distErr, float intErr, float maxSp, float minSp) {
	// TODO Auto-generated constructor stub
	this->kd=d;
	this->kp=p;
	this->ki=i;
	this->distanceErrorOld=distErr;
	this->integratedError=intErr;
	this->maxSpeed=maxSp;
	this->minSpeed=minSp;
}

float PID_horz::getNewValue(float distance,float elapsedTime)
{
	float newTurnSpeed;
	float error=distance;
	if(abs(error)<2)
	{
		newTurnSpeed=0;
		resetErrors();
    return 0;
	}
	float diffE=error-this->distanceErrorOld;
	this->integratedError=this->integratedError+error*elapsedTime;
	this->distanceErrorOld=error;
	newTurnSpeed=this->kp*error+this->ki*this->integratedError+this->kd*diffE;
	if(newTurnSpeed<this->minSpeed)
	{
		newTurnSpeed=this->minSpeed;
	}
	else if(newTurnSpeed>this->maxSpeed)
	{
		newTurnSpeed=this->maxSpeed;
	}
	return newTurnSpeed;
}

void PID_horz::resetErrors()
{
	this->integratedError=0;
	this->distanceErrorOld=0;
}
