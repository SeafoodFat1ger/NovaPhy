#include "novaphy/dynamics/articulated_solver.h"

#include <cmath>

namespace novaphy {

void ArticulatedSolver::step(const Articulation& model,
                              VecXf& q,
                              VecXf& qd,
                              const VecXf& tau,
                              const Vec3f& gravity,
                              float dt) {
    // Forward dynamics: compute accelerations
    VecXf qdd = featherstone::forward_dynamics(model, q, qd, tau, gravity);

    // Semi-implicit Euler: update velocity first, then position
    qd += qdd * dt;

    // Integrate positions
    // For revolute joints: q += qd * dt
    // For free joints: position += v * dt, quaternion += 0.5 * omega * q * dt
    for (int i = 0; i < model.num_links(); ++i) {
        const auto& joint = model.joints[i];
        int qi = model.q_start(i);
        int qdi = model.qd_start(i);

        switch (joint.type) {
            case JointType::Revolute:
                q(qi) += qd(qdi) * dt;
                break;

            case JointType::Fixed:
                break;

            case JointType::Free: {
                // Position: q[0:3] += qd[3:6] * dt  (linear velocity is last 3 in spatial)
                q(qi + 0) += qd(qdi + 3) * dt;
                q(qi + 1) += qd(qdi + 4) * dt;
                q(qi + 2) += qd(qdi + 5) * dt;

                // Quaternion: q[3:7] += 0.5 * omega * q * dt
                // qd[0:3] is angular velocity
                float wx = qd(qdi + 0);
                float wy = qd(qdi + 1);
                float wz = qd(qdi + 2);

                float qx = q(qi + 3);
                float qy = q(qi + 4);
                float qz = q(qi + 5);
                float qw = q(qi + 6);

                // dq/dt = 0.5 * omega * q (quaternion multiplication)
                float dqx = 0.5f * (wx * qw + wz * qy - wy * qz);
                float dqy = 0.5f * (wy * qw - wz * qx + wx * qz);
                float dqz = 0.5f * (wz * qw + wy * qx - wx * qy);
                float dqw = 0.5f * (-wx * qx - wy * qy - wz * qz);

                q(qi + 3) += dqx * dt;
                q(qi + 4) += dqy * dt;
                q(qi + 5) += dqz * dt;
                q(qi + 6) += dqw * dt;
                break;
            }
        }
    }

    // Normalize quaternions
    normalize_quaternions(model, q);

    // Angular velocity damping
    qd *= 0.999f;
}

void ArticulatedSolver::normalize_quaternions(const Articulation& model, VecXf& q) {
    for (int i = 0; i < model.num_links(); ++i) {
        if (model.joints[i].type == JointType::Free) {
            int qi = model.q_start(i);
            float norm = std::sqrt(
                q(qi+3)*q(qi+3) + q(qi+4)*q(qi+4) +
                q(qi+5)*q(qi+5) + q(qi+6)*q(qi+6));
            if (norm > 1e-6f) {
                q(qi+3) /= norm;
                q(qi+4) /= norm;
                q(qi+5) /= norm;
                q(qi+6) /= norm;
            }
        }
    }
}

}  // namespace novaphy
