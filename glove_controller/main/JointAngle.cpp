/*
 * JointAngle.cpp
 *
 *  Created on: 4 mar 2026
 *      Author: karol
 */

#include <iostream>
#include "JointAngle.h"
#include <cmath>
#include "esp_timer.h"

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

// =============================================================================
// Madgwick AHRS filter
//
// Call order per sensor tick:
//   CorrectedData d = processSample(rawAccel, rawMag, rawGyro);
//   mad_filter(d);
//   Quaternion q = getOrientation();   // <- add this accessor if needed
//
// Tune _beta (set in constructor):
//   low  (~0.01) — trust gyroscope more, slower drift correction
//   high (~0.1)  — trust accel/mag more, faster correction but noisier
// =============================================================================

void JointAngle::mad_filter(CorrectedData data, float alpha) {
    
    static Quaternion accel_orient;
    // ── dt ────────────────────────────────────────────────────────────────────
    int64_t now_us = esp_timer_get_time();
    float dt = (float)(now_us - _last_us) * 1e-6f;
    _last_us = now_us;
    if (dt <= 0.f || dt > 0.5f) dt = 0.01f;

    // Local aliases for readability (matches paper notation q1=w q2=x q3=y q4=z)
    const float q1 = _q_est.getQ1();  // w
    const float q2 = _q_est.getQ2();  // x
    const float q3 = _q_est.getQ3();  // y
    const float q4 = _q_est.getQ4();  // z

    Quaternion q_dot_gyro = _q_est.derivative_body(
        data.gyro[0], data.gyro[1], data.gyro[2]
    );

    Quaternion orient_gyro((_q_est*0.5f) + (q_dot_gyro*dt));

    // =========================================================================
    // STEP 2 — Accelerometer gradient  ∇f_g = J_g^T · f_g
    //
    //   Reference gravity in earth frame: [0, 0, 1]
    //   f_g = (q* ⊗ [0,0,1] ⊗ q) − a_normalised   (closed form below)
    // =========================================================================
    Quaternion earth_ref_accel(0, 1, 0, 0);
    Quaternion accel_meas(0, data.accel[0], data.accel[1], data.accel[2]);
    float u_accel = this->get_beta() * dt;

    Quaternion f_func((accel_orient.conjugate() * earth_ref_accel * accel_orient) - accel_meas);

    accel_orient = accel_orient - ((f_func.jacobianGradient(1e-3f))*(1.0f/f_func.jacobianGradient(1e-3).norm()))* u_accel;

    // =========================================================================
    // STEP 3 — Magnetometer gradient  ∇f_b = J_b^T · f_b
    //
    //   h  = q ⊗ [0,mx,my,mz] ⊗ q*       (mag rotated to earth frame)
    //   b  = [0, sqrt(hx²+hy²), 0, hz]    (flatten x/y → remove yaw ambiguity)
    //   f_b = (q* ⊗ [0,bx,0,bz] ⊗ q) − m_normalised   (closed form below)
    // =========================================================================
    Quaternion earth_ref_mag(0,0.5,0,0.5);
    Quaternion mag_measure(0, data.mag[0], data.mag[1], data.mag[2]);
    
    Quaternion mu_mag = q_dot_gyro * alpha * dt;


    // =========================================================================
    // FINAL STEP — Fuse gyro integration with gradient correction
    //
    //   ∇f       = combined gradient (accel + mag)
    //   q_est,t  = normalise( q_est + (q_dot_gyro − β · ∇f/|∇f|) · dt )
    // =========================================================================
    
}

//Accessors
int JointAngle::getFinger() const {
    return finger;
}

CalibrationState JointAngle::getState() const {
    return calState;
}