#include "novaphy/core/joint.h"

#include <cmath>

namespace novaphy {

Transform Joint::joint_transform(const float* q) const {
    switch (type) {
        case JointType::Revolute: {
            float angle = q[0];
            return Transform::from_axis_angle(axis, angle);
        }
        case JointType::Fixed:
            return Transform::identity();
        case JointType::Free: {
            // q = [px, py, pz, qx, qy, qz, qw]
            Vec3f pos(q[0], q[1], q[2]);
            Quatf rot(q[6], q[3], q[4], q[5]);  // Eigen: w,x,y,z
            rot.normalize();
            return Transform(pos, rot);
        }
    }
    return Transform::identity();
}

void Joint::motion_subspace(SpatialVector* S_cols) const {
    switch (type) {
        case JointType::Revolute:
            // Single column: rotation about axis
            S_cols[0] = make_spatial(axis, Vec3f::Zero());
            break;
        case JointType::Fixed:
            // No columns
            break;
        case JointType::Free:
            // 6 columns: identity (angular first, then linear)
            for (int i = 0; i < 6; ++i) {
                S_cols[i] = SpatialVector::Zero();
                S_cols[i](i) = 1.0f;
            }
            break;
    }
}

}  // namespace novaphy
