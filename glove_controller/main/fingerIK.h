/*
 * FingerIK.h
 *
 * Quaternion Jacobian-Transpose IK solver for PIP & DIP joint estimation.
 *
 *  Created on: 9 mar 2026
 *      Author: edoardo
 */

#ifndef MAIN_FINGERIK_H_
#define MAIN_FINGERIK_H_

#include "Quaternion.h"
#include <cmath>


// Constraint mode selector
//   COUPLING_PRIOR  — θ_DIP ≈ k · θ_PIP  (Lee & Kunii 1995, soft penalty)
//   FLEX_STRIP      — θ_PIP pinned to flex-sensor reading; only θ_DIP solved

enum class ConstraintMode {
	CouplingPrior,
	FlexStrip
};

class FingerIK {
private:
//Persistent state (warm-started across ticks)
	float _theta_pip;
	float _theta_dip;

//Constraint configuration
	ConstraintMode _mode;
	float _theta_pip_measured;          // used only in FlexStrip mode

//Solver tunables
	float _alpha;                       // gradient step size
	int   _max_iter;                    // iterations per tick
	float _epsilon;                     // convergence threshold

	float _k_couple;                    // DIP/PIP biomechanical ratio
	float _lambda_couple;               // coupling soft-constraint strength

	float _pip_min;                     // joint limits [rad]
	float _pip_max;
	float _dip_min;
	float _dip_max;

//Internal helpers
	// Builds q_rot_y(θ) = Quaternion(cos(θ/2), 0, sin(θ/2), 0)
	static Quaternion qRotY(float theta);
	// Clamps value to [lo, hi]
	static float clamp(float val, float lo, float hi);

public:
	FingerIK();
	virtual ~FingerIK();
//Constraint interface
	//  CouplingPrior : param is ignored (coupling ratio set via setKCouple)
	//  FlexStrip     : param = current θ_PIP reading from flex sensor [rad]
	void setConstraintMode(ConstraintMode mode, float param = 0.0f);

// Main solver entry point 
	//  Call once per sensor tick.
	//    q_hand_base   : absolute orientation of the metacarpal frame (wrist IMU)
	//    theta_mcp     : MCP angle [rad] (from JointAngle pipeline)
	//    q_tip_measured: Madgwick _q_est from fingertip IMU
	void update(const Quaternion& q_hand_base,
				float              theta_mcp,
				const Quaternion&  q_tip_measured);

//Accessors
	float getPIP() const { return _theta_pip*57.29578f; }
	float getDIP() const { return _theta_dip* 57.29578f; }

//Tunable setters
	void setAlpha(float v)        { _alpha = v; }
	void setMaxIter(int v)        { _max_iter = v; }
	void setEpsilon(float v)      { _epsilon = v; }
	void setKCouple(float v)      { _k_couple = v; }
	void setLambdaCouple(float v) { _lambda_couple = v; }
	void setPIPLimits(float lo, float hi) { _pip_min = lo; _pip_max = hi; }
	void setDIPLimits(float lo, float hi) { _dip_min = lo; _dip_max = hi; }
};

#endif /* MAIN_FINGERIK_H_ */