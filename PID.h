/*
 * PID.h
 *
 *  Created on: Nov 9, 2016
 *      Author: Admin
 */

#ifndef PID_H_
#define PID_H_

class PID {
	public:
		PID(float d, float p, float i, float headErr, float intErr, float maxSp, float minSp);
		float getNewValue(float currHeading, float desiredHeading,float elapsedTime);
		void resetErrors();
	private:
		float kd;
		float kp;
		float ki;
		float headingErrorOld;
		float integratedError;
		float maxSpeed;
		float minSpeed;
};

#endif /* PID_H_ */
