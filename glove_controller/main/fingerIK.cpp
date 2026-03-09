/*
 * FingerIK.cpp
 *
 *  Created on: 9 mar 2026
 *      Author: edoardo
 */

#include "fingerIK.h"

Quaternion FingerIK::qRotY(float theta) {
	float half = theta * 0.5f;
	return Quaternion(std::cos(half), 0.0f, std::sin(half), 0.0f);
}

float FingerIK::clamp(float val, float lo, float hi) {
	if (val < lo) return lo;
	if (val > hi) return hi;
	return val;
}

// Construction / destruction

FingerIK::FingerIK()
	: _theta_pip(0.0f)
	, _theta_dip(0.0f)
	, _mode(ConstraintMode::CouplingPrior)
	, _theta_pip_measured(0.0f)
	, _alpha(0.5f)
	, _max_iter(5)
	, _epsilon(1e-4f)
	, _k_couple(0.67f)
	, _lambda_couple(0.3f)
	, _pip_min(0.0f)
	, _pip_max(1.745f)        // 100deg
	, _dip_min(0.0f)
	, _dip_max(1.222f)        // 70deg
{
}

FingerIK::~FingerIK() {
}

// Constraint mode

void FingerIK::setConstraintMode(ConstraintMode mode, float param) {
	_mode = mode;
	if (mode == ConstraintMode::FlexStrip) {
		_theta_pip_measured = param;
	}
}


// Jacobian-Transpose IK solver

void FingerIK::update(const Quaternion& q_hand_base,
					  float              theta_mcp,
					  const Quaternion&  q_tip_measured)
{

	Quaternion q_mcp = q_hand_base * qRotY(theta_mcp);

	Quaternion q_rel = q_mcp.inverse() * q_tip_measured;

	if (_mode == ConstraintMode::FlexStrip) {
		//TODO
		return;
	}

	for (int iter = 0; iter < _max_iter; iter++) {

		Quaternion q_fk = qRotY(_theta_pip) * qRotY(_theta_dip);

		Quaternion q_err = q_fk.inverse() * q_rel;

		if (q_err.getQ1() < 0.0f) {
			q_err = -q_err;
		}

		float ex = q_err.getQ2();
		float ey = q_err.getQ3();
		float ez = q_err.getQ4();

		float err_norm = std::sqrt(ex * ex + ey * ey + ez * ez);
		if (err_norm < _epsilon) break;

		float grad_pip = ey;
		float grad_dip = ey;

		float coupling_err = _theta_dip - _k_couple * _theta_pip;
		grad_pip += _lambda_couple * _k_couple * coupling_err;
		grad_dip -= _lambda_couple * coupling_err;

		_theta_pip += _alpha * grad_pip;
		_theta_dip += _alpha * grad_dip;

		_theta_pip = clamp(_theta_pip, _pip_min, _pip_max);
		_theta_dip = clamp(_theta_dip, _dip_min, _dip_max);
	}
}