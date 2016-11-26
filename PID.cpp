/*
 * PID.cpp
 *
 *  Created on: Nov 9, 2016
 *      Author: Admin
 */

#include "PID.h"
#include "stdlib.h"

using namespace std;

PID::PID(float d, float p, float i, float headErr, float intErr, float maxSp, float minSp) {
	// TODO Auto-generated constructor stub
	this->kd=d;
	this->kp=p;
	this->ki=i;
	this->headingErrorOld=headErr;
	this->integratedError=intErr;
	this->maxSpeed=maxSp;
	this->minSpeed=minSp;
}

float PID::getNewValue(float currHeading, float desiredHeading,float elapsedTime)
{
	float newTurnSpeed;
	float error=desiredHeading-currHeading;
	if(abs(error)<10)
	{
		newTurnSpeed=0;
		resetErrors();
    return 0;
	}
	float diffE=error-this->headingErrorOld;
	this->integratedError=this->integratedError+error*elapsedTime;
	this->headingErrorOld=error;
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

void PID::resetErrors()
{
	this->integratedError=0;
	this->headingErrorOld=0;
}

