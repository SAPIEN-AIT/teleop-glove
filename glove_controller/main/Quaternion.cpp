/*
 * Quaternion.cpp
 *
 *  Created on: 7 mar 2026
 *      Author: Karol Wickel
 */

#include "Quaternion.h"
#include <cmath>
#include "esp_log.h"
static const char* TAG = "Quaternion";


Quaternion::Quaternion(float q1, float q2, float q3, float q4){
	this->_q1 = q1;
	this->_q2 = q2;
	this->_q3 = q3;
	this->_q4 = q4;
}

Quaternion::~Quaternion() {
	// TODO Auto-generated destructor stub
}

Quaternion::Quaternion(const Quaternion &other) {
    _q1 = other._q1;
    _q2 = other._q2;
    _q3 = other._q3;
    _q4 = other._q4;
}

Quaternion::Quaternion(Quaternion &&other) {
    _q1 = other._q1;
    _q2 = other._q2;
    _q3 = other._q3;
    _q4 = other._q4;
    other._q1 = 0.0f;
    other._q2 = 0.0f;
    other._q3 = 0.0f;
    other._q4 = 0.0f;
}

Quaternion& Quaternion::operator=(const Quaternion &other) {
	    if (this != &other) {
        _q1 = other._q1;
        _q2 = other._q2;
        _q3 = other._q3;
        _q4 = other._q4;
    }
    return *this;
}

Quaternion& Quaternion::operator=(Quaternion &&other) {
	    if (this != &other) {
        _q1 = other._q1;
        _q2 = other._q2;
        _q3 = other._q3;
        _q4 = other._q4;
        other._q1 = 0.0f;
        other._q2 = 0.0f;
        other._q3 = 0.0f;
        other._q4 = 0.0f;
    }
    return *this;
}

Quaternion Quaternion::operator+(const Quaternion& other) const {
    return Quaternion(
        _q1 + other._q1,
        _q2 + other._q2,
        _q3 + other._q3,
        _q4 + other._q4
    );
}

Quaternion Quaternion::operator-(const Quaternion& other) const {
    return Quaternion(
        _q1 - other._q1,
        _q2 - other._q2,
        _q3 - other._q3,
        _q4 - other._q4
    );
}

Quaternion& Quaternion::operator+=(const Quaternion& other) {
    _q1 += other._q1;
    _q2 += other._q2;
    _q3 += other._q3;
    _q4 += other._q4;
    return *this;
}

Quaternion& Quaternion::operator-=(const Quaternion& other) {
    _q1 -= other._q1;
    _q2 -= other._q2;
    _q3 -= other._q3;
    _q4 -= other._q4;
    return *this;
}

// Unary minus — negate all components
Quaternion Quaternion::operator-() const {
    return Quaternion(-_q1, -_q2, -_q3, -_q4);
}

Quaternion Quaternion::operator*(float other){
	auto placeholder = Quaternion(this->_q1 * other, this->_q2 * other,
								  this->_q3 * other, this->_q4 * other);
	return placeholder;
}

Quaternion Quaternion::operator*(const  Quaternion& other) const{
	return {
		_q1*other._q1 - _q2*other._q2 - _q3*other._q3 - _q4*other._q4,
		_q1*other._q2 + _q2*other._q1 + _q3*other._q4 - _q4*other._q3,
		_q1*other._q3 - _q2*other._q4 + _q3*other._q1 + _q4*other._q2,
		_q1*other._q4 + _q2*other._q3 - _q3*other._q2 + _q4*other._q1
	};
}

Quaternion Quaternion::rotate_by(Quaternion other) {
    return other * (*this) * other.inverse();
}

Quaternion Quaternion::conjugate() const{
	return (Quaternion){_q1, -_q2, -_q3, -_q4};	
}

Quaternion Quaternion::inverse() const{
	float n2 = _q1*_q1 + _q2*_q2 + _q3*_q3 + _q4*_q4;
	if(n2<1e-15){
		ESP_LOGE(TAG, "cannot invert a zero quaternion, returning identity");
        return Quaternion(1, 0, 0, 0);
	}
	return conjugate() * (float)(1.0/n2);
}

float Quaternion::norm() const {
    return std::sqrt(_q1*_q1 + _q2*_q2 + _q3*_q3 + _q4*_q4);
}


Quaternion Quaternion::normalized(){
	float n = norm();
	if(n<1e-15){
		ESP_LOGE(TAG, "cannot normalize a zero quaternion, returning identity");
        return Quaternion(1, 0, 0, 0);
	}
	
	return *this * (float)(1.0/n);
}



Quaternion Quaternion::derivative_body(float wx, float wy, float wz) {
    Quaternion omega(0.0f, wx, wy, wz);
    Quaternion qdot = (*this) * omega;
    return qdot * 0.5f;
}


Quaternion Quaternion::derivative_world(float wx, float wy, float wz){
    Quaternion omega(0.0f, wx, wy, wz);
    Quaternion qdot = omega * (*this);
    return qdot * 0.5f;
}

// Integrate orientation one step forward using Euler method:
//      q(t+dt) = q + dq/dt * dt
// then renormalize to keep unit quaternion	
Quaternion Quaternion::integrate(float wx, float wy, float wz, float dt){
    Quaternion qdot = derivative_body(wx, wy, wz);
    Quaternion next = *this + qdot * dt;
    return next.normalized();
}

float Quaternion::joint_angle() const {
    float y = 2.0f * (_q1*_q4 + _q2*_q3);
    float x = 1.0f - (_q3*_q3 + _q4*_q4);
    return std::atan2(y, x) * 57.29578f;
}


Quaternion Quaternion::jacobianGradient(float eps) const {
    Quaternion dfdq1 = ((Quaternion(this->_q1+eps, this->_q2, this->_q3, this->_q4).normalized()) - *this) * (1.f/eps);
    Quaternion dfdq2 = ((Quaternion(this->_q1, this->_q2+eps, this->_q3, this->_q4).normalized()) - *this) * (1.f/eps);
    Quaternion dfdq3 = ((Quaternion(this->_q1, this->_q2, this->_q3+eps, this->_q4).normalized()) - *this) * (1.f/eps);
    Quaternion dfdq4 = ((Quaternion(this->_q1, this->_q2, this->_q3, this->_q4+eps).normalized()) - *this) * (1.f/eps);

    return Quaternion(
        dfdq1._q1*this->_q1 + dfdq1._q2*_q2 + dfdq1._q3*_q3 + dfdq1._q4*_q4,
        dfdq2._q1*this->_q1 + dfdq2._q2*_q2 + dfdq2._q3*_q3 + dfdq2._q4*_q4,
        dfdq3._q1*this->_q1 + dfdq3._q2*_q2 + dfdq3._q3*_q3 + dfdq3._q4*_q4,
        dfdq4._q1*this->_q1 + dfdq4._q2*_q2 + dfdq4._q3*_q3 + dfdq4._q4*_q4
    );
}   