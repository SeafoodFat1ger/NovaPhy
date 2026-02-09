#pragma once

#include "novaphy/math/math_types.h"
#include "novaphy/math/spatial.h"

namespace novaphy {

/// Joint types for articulated bodies
enum class JointType {
    Revolute,  // 1 DOF: rotation about a single axis
    Fixed,     // 0 DOF: rigid attachment
    Free,      // 6 DOF: free-floating (root body)
};

/// Joint definition for an articulated body link.
struct Joint {
    JointType type = JointType::Revolute;
    Vec3f axis = Vec3f(0, 0, 1);  // rotation axis (for revolute joints)
    int parent = -1;               // parent link index (-1 = world)

    /// Transform from parent body frame to joint frame (constant offset)
    Transform parent_to_joint = Transform::identity();

    /// Number of position DOFs for this joint type
    int num_q() const {
        switch (type) {
            case JointType::Revolute: return 1;
            case JointType::Fixed:    return 0;
            case JointType::Free:     return 7;  // pos(3) + quat(4)
        }
        return 0;
    }

    /// Number of velocity DOFs for this joint type
    int num_qd() const {
        switch (type) {
            case JointType::Revolute: return 1;
            case JointType::Fixed:    return 0;
            case JointType::Free:     return 6;  // linear(3) + angular(3)
        }
        return 0;
    }

    /// Compute the joint transform X_J(q) given joint coordinates.
    /// For revolute: rotation about axis by angle q[0]
    /// For fixed: identity
    /// For free: translation + rotation from q
    Transform joint_transform(const float* q) const;

    /// Compute the motion subspace matrix S (6 x nv).
    /// Returns columns as spatial vectors.
    /// For revolute: single column [axis; 0]
    /// For free: 6x6 identity
    void motion_subspace(SpatialVector* S_cols) const;
};

}  // namespace novaphy
