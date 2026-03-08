/*
 * JointAngle.h
 *
 *  Created on: 4 mar 2026
 *      Author: karol, edoardo
 */

#ifndef MAIN_JOINTANGLE_H_
#define MAIN_JOINTANGLE_H_

#include "Quaternion.h"
#include <cstdint>

enum class CalibrationState {
    Idle,
    CollectingData,
    Running,
    Error
};

struct CorrectedData {
	float accel[3]; // the corrected acceleration
	float mag[3];	// quaternions for magnetometer mck = [qb, qc, qd]
	float gyro[3];	// raw data [wx, wy, wz]
};

class JointAngle {
private:
//Attributes
	int finger; 
	//calibration	
	CalibrationState calState;
	float accelBias[3];
    Quaternion headingQuat;
	float accelAccum[3];
    float magAccum[3];
    int sampleCount;
    int targetSamples;
	float beta;
	Quaternion _q_est;
	int64_t _last_us;


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
	
	void mad_filter(CorrectedData data, float alpha);

	//Accessors
	int getFinger() const;
	CalibrationState getState() const;
	float get_beta() const {return beta;};
};

#endif /* MAIN_JOINTANGLE_H_ */
