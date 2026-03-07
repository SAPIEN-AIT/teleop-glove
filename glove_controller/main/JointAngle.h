/*
 * JointAngle.h
 *
 *  Created on: 4 mar 2026
 *      Author: karol, edoardo
 */

#ifndef MAIN_JOINTANGLE_H_
#define MAIN_JOINTANGLE_H_

#include "Quaternion.h"

enum class CalibrationState {
    Idle,
    CollectingData,
    Running,
    Error
};

struct CorrectedData {
	float accel[3];
	float mag[3];
	float gyro[3];
};

class JointAngle {
private:
//Attributes
	int finger; 
	//calibration	
	CalibrationState calState;
	float accelBias[3];
    float headingQuat[4];
	float accelAccum[3];
    float magAccum[3];
    int sampleCount;
    int targetSamples;

//Methods
	//calibration
	void computeAccelBias();
    void computeHeadingQuat();
    void calibrateAccel(const float rawAccel[3], float corrected[3]);
    void calibrateMag(const float rawMag[3], const float accel[3], float corrected[3]);

public:
	JointAngle();
	virtual ~JointAngle();

	//calibration
	void beginCalibration(int numSamples);
    void addCalibrationSample(const float accel[3], const float mag[3]);
	CorrectedData processSample(const float rawAccel[3], const float rawMag[3], const float rawGyro[3]);

	void filter();
	
	//Accessors
	int getFinger();
	CalibrationState getState();
};

#endif /* MAIN_JOINTANGLE_H_ */
