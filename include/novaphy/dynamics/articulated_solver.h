#pragma once

#include "novaphy/core/articulation.h"
#include "novaphy/dynamics/featherstone.h"
#include "novaphy/math/math_types.h"

namespace novaphy {

/// Articulated body solver: wraps Featherstone forward dynamics with integration.
class ArticulatedSolver {
public:
    ArticulatedSolver() = default;

    /// Step the articulated body forward in time.
    /// Updates q and qd in-place using semi-implicit Euler.
    ///
    /// @param model  Articulation model
    /// @param q      Generalized positions (updated in-place)
    /// @param qd     Generalized velocities (updated in-place)
    /// @param tau    Applied joint torques
    /// @param gravity Gravity vector
    /// @param dt     Time step
    void step(const Articulation& model,
              VecXf& q,
              VecXf& qd,
              const VecXf& tau,
              const Vec3f& gravity,
              float dt);

private:
    /// Normalize quaternions in q for free joints
    void normalize_quaternions(const Articulation& model, VecXf& q);
};

}  // namespace novaphy
