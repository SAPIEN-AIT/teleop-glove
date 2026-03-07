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

    headingQuat = Quaternion(1.0f, 0.0f, 0.0f, 0.0f);
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

void JointAngle::computeHeadingQuat() {
    float magAvg[3];
    float accelAvg[3];
    for (int i = 0; i < 3; i++) {
        magAvg[i] = magAccum[i] / targetSamples;
        accelAvg[i] = accelAccum[i] / targetSamples;
    }

    float roll  = std::atan2(accelAvg[1], accelAvg[2]);
    float pitch = std::atan2(-accelAvg[0],
                  std::sqrt(accelAvg[1] * accelAvg[1] + accelAvg[2] * accelAvg[2]));

    float sinR = std::sin(roll);
    float cosR = std::cos(roll);
    float sinP = std::sin(pitch);
    float cosP = std::cos(pitch);

    float mGx = magAvg[0] * cosP
              + magAvg[1] * sinP * sinR
              + magAvg[2] * sinP * cosR;

    float mGy = magAvg[1] * cosR
              - magAvg[2] * sinR;

    float heading = std::atan2(mGy, mGx);

    headingQuat = Quaternion(
		std::cos(heading / 2.0f),
		0.0f,
		0.0f,
		std::sin(heading / 2.0f)
	);

}

void JointAngle::calibrateMag(const float rawMag[3], const float accel[3], float corrected[3]) {

    float roll  = std::atan2(accel[1], accel[2]);
    float pitch = std::atan2(-accel[0],
                  std::sqrt(accel[1] * accel[1] + accel[2] * accel[2]));

    float sinR = std::sin(roll);
    float cosR = std::cos(roll);
    float sinP = std::sin(pitch);
    float cosP = std::cos(pitch);

    float mGx = rawMag[0] * cosP
              + rawMag[1] * sinP * sinR
              + rawMag[2] * sinP * cosR;

    float mGy = rawMag[1] * cosR
              - rawMag[2] * sinR;

    float mGz = -rawMag[0] * sinP
              + rawMag[1] * sinR * cosP
              + rawMag[2] * cosR * cosP;

    Quaternion qm(0.0f, mGx, mGy, mGz);

    Quaternion qmCorrected = (headingQuat.conjugate() * qm) * headingQuat;

    corrected[0] = qmCorrected.getQ2();
    corrected[1] = qmCorrected.getQ3();
    corrected[2] = qmCorrected.getQ4();
}

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

//Accessors
int JointAngle::getFinger() const {
    return finger;
}

CalibrationState JointAngle::getState() const {
    return calState;
}