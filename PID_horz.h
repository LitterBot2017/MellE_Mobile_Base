/*
 * PID.h
 *
 *  Created on: Nov 9, 2016
 *      Author: Admin
 */

#ifndef PID_HORZ_H_
#define PID_HORZ_H_

class PID_horz {
	public:
		PID_horz(float d, float p, float i, float distErr, float intErr, float maxSp, float minSp);
		float getNewValue(float distance,float elapsedTime);
		void resetErrors();
	private:
		float kd;
		float kp;
		float ki;
		float distanceErrorOld;
		float integratedError;
		float maxSpeed;
		float minSpeed;
};

#endif /* PID_HORZ_H_ */