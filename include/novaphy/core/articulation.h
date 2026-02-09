#pragma once

#include <vector>

#include "novaphy/core/body.h"
#include "novaphy/core/joint.h"
#include "novaphy/math/math_types.h"
#include "novaphy/math/spatial.h"

namespace novaphy {

/// An articulated body: a kinematic tree of rigid bodies connected by joints.
/// Links are topologically sorted so parent index < child index.
struct Articulation {
    std::vector<Joint> joints;           // joint[i] connects link i to its parent
    std::vector<RigidBody> bodies;       // rigid body properties for each link
    std::vector<SpatialMatrix> I_body;   // spatial inertia for each link (body frame)

    /// Total number of links
    int num_links() const { return static_cast<int>(joints.size()); }

    /// Total position DOFs (sum of all joints' num_q)
    int total_q() const {
        int n = 0;
        for (const auto& j : joints) n += j.num_q();
        return n;
    }

    /// Total velocity DOFs (sum of all joints' num_qd)
    int total_qd() const {
        int n = 0;
        for (const auto& j : joints) n += j.num_qd();
        return n;
    }

    /// Position DOF offset for link i
    int q_start(int link) const {
        int offset = 0;
        for (int i = 0; i < link; ++i) offset += joints[i].num_q();
        return offset;
    }

    /// Velocity DOF offset for link i
    int qd_start(int link) const {
        int offset = 0;
        for (int i = 0; i < link; ++i) offset += joints[i].num_qd();
        return offset;
    }

    /// Build spatial inertia matrices from body properties
    void build_spatial_inertias() {
        I_body.resize(num_links());
        for (int i = 0; i < num_links(); ++i) {
            I_body[i] = spatial_inertia_matrix(
                bodies[i].mass, bodies[i].com, bodies[i].inertia);
        }
    }
};

}  // namespace novaphy
