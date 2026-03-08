/*
 * Quaternion.h
 *
 *  Created on: 7 mar 2026
 *      Author: karol
 */

#ifndef MAIN_QUATERNION_H_
#define MAIN_QUATERNION_H_

class Quaternion {
private:
	float _q1, _q2, _q3, _q4;
public:
	Quaternion(float q1 = 0, float q2 = 0, float q3 = 0, float q4 = 0);
	virtual ~Quaternion();
	Quaternion(const Quaternion &other);
	Quaternion(Quaternion &&other);
	Quaternion& operator=(const Quaternion &other);
	Quaternion& operator=(Quaternion &&other);
	Quaternion  operator+(const Quaternion& other) const;
    Quaternion  operator-(const Quaternion& other) const;
    Quaternion  operator-() const;
    Quaternion& operator+=(const Quaternion& other);
    Quaternion& operator-=(const Quaternion& other);
	Quaternion operator*(float other);
	Quaternion operator*(const Quaternion& other) const;
	Quaternion inverse()const;
	Quaternion rotate_by(Quaternion other);
	Quaternion conjugate() const;
	float norm() const;
	Quaternion normalized();
	float joint_angle() const;
	float getQ1() const { return _q1; }
	float getQ2() const { return _q2; }
	float getQ3() const { return _q3; }
	float getQ4() const { return _q4; }
	void setQ1(float q1) {this->_q1 = q1;}
	void setQ2(float q2) {this->_q2 = q2;}
	void setQ3(float q3) {this->_q3 = q3;}
	void setQ4(float q4) {this->_q4 = q4;}
		
	//Calculus
    Quaternion derivative_body(float wx, float wy, float wz);
    Quaternion derivative_world(float wx, float wy, float wz) ;
    Quaternion integrate(float wx, float wy, float wz, float dt);

	Quaternion jacobianGradient(float eps) const;
};

#endif /* MAIN_QUATERNION_H_ */
