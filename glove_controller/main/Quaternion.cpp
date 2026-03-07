/*
 * Quaternion.cpp
 *
 *  Created on: 7 mar 2026
 *      Author: karol
 */

#include "Quaternion.h"

Quaternion::Quaternion() {
	// TODO Auto-generated constructor stub

}

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
	// TODO Auto-generated constructor stub

}

Quaternion::Quaternion(Quaternion &&other) {
	// TODO Auto-generated constructor stub

}

Quaternion& Quaternion::operator=(const Quaternion &other) {
	// TODO Auto-generated method stub

}

Quaternion& Quaternion::operator=(Quaternion &&other) {
	// TODO Auto-generated method stub
	
}

Quaternion Quaternion::operator*(float &other){
	auto placeholder = Quaternion(this->_q1 * other, this->_q2 * other,
								  this->_q3 * other, this->_q4 * other);
	return placeholder;
}

Quaternion Quaternion::operator*(Quaternion& other){
	
}
