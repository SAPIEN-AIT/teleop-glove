# FingerIK PIP & DIP Joint Estimator

Quaternion Jacobian-Transpose IK solver that estimates the PIP and DIP flexion angles of a finger using a single fingertip IMU, given the known hand orientation and MCP angle.

## Problem

The finger kinematic chain downstream of MCP is:

```
q_tip = q_hand ⊗ q_y(θ_MCP) ⊗ q_y(θ_PIP) ⊗ q_y(θ_DIP)
```

where `q_y(θ) = (cos θ/2,  0,  sin θ/2,  0)` is a pure Y-axis rotation.

We measure `q_tip` from the distal-phalanx IMU (Madgwick AHRS) and need to recover θ\_PIP and θ\_DIP.

### Observability gap

Both PIP and DIP rotate about the **same local Y-axis**. The angular-velocity Jacobian is:

```
         ┌ 0  0 ┐
J_ω  =   │ 1  1 │    ← rank 1
         └ 0  0 ┘
```

This means the tip orientation only constrains **θ\_PIP + θ\_DIP**, not individual values. A secondary constraint is mandatory.

## Algorithm

**Step 1: Remove wrist + MCP.** Build the MCP frame and express the measured tip orientation relative to it:

```
q_mcp = q_hand ⊗ q_y(θ_MCP)
q_rel = q_mcp⁻¹ ⊗ q_tip_measured
```

**Step 2: Iterative Jacobian-Transpose loop** on the residual 2-DoF chain (warm-started from previous tick):

```
for each iteration:
    q_fk  = q_y(θ_PIP) ⊗ q_y(θ_DIP)           // forward kinematics
    q_err = q_fk⁻¹ ⊗ q_rel                      // orientation error
    if q_err.w < 0: q_err = −q_err               // short-path convention
    e     = [q_err.x,  q_err.y,  q_err.z]        // error vector

    break if ‖e‖ < ε

    J^T · e = [e_y,  e_y]                        // both axes are ŷ

    apply constraint gradient (see below)
    θ += α · grad
    clamp to joint limits
```

**Step 3: Constraint injection** (breaks the rank-1 degeneracy):

| Mode | Source | Effect |
|------|--------|--------|
| **CouplingPrior** | Biomechanical law θ\_DIP ≈ k · θ\_PIP (Lee & Kunii 1995, k ≈ 0.67) | Soft penalty added to gradient: `C = θ_DIP − k·θ_PIP`, `grad_PIP += λ·k·C`, `grad_DIP −= λ·C` |
| **FlexStrip** | Flex sensor on middle phalanx measures θ\_PIP directly | θ\_PIP pinned; θ\_DIP solved analytically via `2·atan2(q.y, q.w)` from the residual quaternion. No iteration needed. |

## Defaults

| Parameter | Value | Meaning |
|-----------|-------|---------|
| α | 0.5 | Gradient step size |
| max_iter | 5 | Iterations per tick |
| ε | 1e-4 | Convergence threshold |
| k | 0.67 | DIP/PIP coupling ratio |
| λ | 0.3 | Coupling constraint strength |
| PIP limits | 0 – 1.745 rad (100°) | Joint range |
| DIP limits | 0 – 1.222 rad (70°) | Joint range |

## Example usage

```cpp
#include "FingerIK.h"
#include "JointAngle.h"

// Init (once) — no heap allocation after this
JointAngle wristIMU, mcpIMU, tipIMU;
FingerIK   indexIK;

// Optional: switch to flex strip mode when hardware is present
// indexIK.setConstraintMode(ConstraintMode::FlexStrip, flexReading);

// Per tick (~200 Hz)
CorrectedData wd = wristIMU.procesSample(rawAccW, rawMagW, rawGyroW);
CorrectedData md = mcpIMU.procesSample(rawAccM, rawMagM, rawGyroM);
CorrectedData td = tipIMU.procesSample(rawAccT, rawMagT, rawGyroT);

Quaternion q_hand = wristIMU.mad_filter(wd, alpha, gamma);
Quaternion q_mcp  = mcpIMU.mad_filter(md, alpha, gamma);
Quaternion q_tip  = tipIMU.mad_filter(td, alpha, gamma);

// MCP angle from existing pipeline (returns degrees)
float theta_mcp_rad = JointAngle::computeJointAngle(q_hand, q_mcp, q_ref, 1)
                      * 0.0174533f;

// Solve PIP + DIP
indexIK.update(q_hand, theta_mcp_rad, q_tip);

float pip_deg = indexIK.getPIP();   // degrees (post your rad→deg edit)
float dip_deg = indexIK.getDIP();
```

## References

1. S. Chiaverini and B. Siciliano, "The unit quaternion: a useful tool for inverse kinematics of robot manipulators," *Systems Analysis Modelling Simulation*, vol. 35, pp. 45–60, 1999.
2. B. Siciliano, L. Sciavicco, L. Villani, and G. Oriolo, *Robotics: Modelling, Planning and Control*, Springer, 2009, Ch. 3–4.
3. J. Lee and T. L. Kunii, "Model-based analysis of hand posture," *IEEE CG&A*, vol. 15, no. 6, pp. 77–86, 1995.