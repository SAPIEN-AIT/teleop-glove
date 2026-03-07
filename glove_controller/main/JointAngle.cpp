/*
 * JointAngle.cpp
 *
 *  Created on: 4 mar 2026
 *      Author: karol
 */

#include <iostream>
#include "JointAngle.h"

JointAngle::JointAngle() {
    calState = CalibrationState::Idle;
    sampleCount = 0;
    targetSamples = 0;

    for (int i = 0; i < 3; i++) {
        accelBias[i] = 0.0f;
        accelAccum[i] = 0.0f;
        magAccum[i] = 0.0f;
    }

    for (int i = 0; i < 4; i++) {
        headingQuat[i] = 0.0f;
    }
}

//Calibration
void JointAngle::beginCalibration(int numSamples) {
    targetSamples = numSamples;
    sampleCount = 0;

    for (int i = 0; i < 3; i++) {
        accelAccum[i] = 0.0f;
        magAccum[i] = 0.0f;
    }

    calState = CalibrationState::CollectingData;
}

void JointAngle::addCalibrationSample(const float accel[3], const float mag[3]) {
    if (calState != CalibrationState::CollectingData) {
        return;
    }

    for (int i = 0; i < 3; i++) {
        accelAccum[i] += accel[i];
        magAccum[i] += mag[i];
    }

    sampleCount++;

    if (sampleCount >= targetSamples) {
        computeAccelBias();
        computeHeadingQuat();
        calState = CalibrationState::Running;
    }
}
void JointAngle::computeAccelBias() {
    for (int i = 0; i < 3; i++) {
        accelBias[i] = accelAccum[i] / targetSamples;
    }
}

void JointAngle::calibrateAccel(const float rawAccel[3], float corrected[3]) {
    for (int i = 0; i < 3; i++) {
        corrected[i] = rawAccel[i] - accelBias[i];
    }
}

void JointAngle::calibrateMag(const float rawMag[3], const float accel[3], float corrected[3]) {
	//TODO

};

CorrectedData JointAngle::processSample(const float rawAccel[3], const float rawMag[3], const float rawGyro[3]) {
    CorrectedData out;

    calibrateAccel(rawAccel, out.accel);
    calibrateMag(rawMag, out.accel, out.mag);

    for (int i = 0; i < 3; i++) {
        out.gyro[i] = rawGyro[i];
    }

    return out;
}

JointAngle::~JointAngle() {
	// TODO Auto-generated destructor stub
}

void JointAngle::filter(){
	
}