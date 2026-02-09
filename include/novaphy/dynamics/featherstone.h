#pragma once

#include <vector>

#include "novaphy/core/articulation.h"
#include "novaphy/math/math_types.h"
#include "novaphy/math/spatial.h"

namespace novaphy {

/// Featherstone algorithms for articulated body dynamics.
/// Convention: [angular; linear] spatial vectors.
/// All algorithms operate on a single Articulation.
namespace featherstone {

/// Forward kinematics: compute link transforms from generalized coordinates q.
///
/// @param model  Articulation model (joints, bodies)
/// @param q      Generalized position coordinates (length = model.total_q())
/// @param X_J    Output: joint transforms (one per link)
/// @param X_up   Output: transform from link i to parent (one per link)
/// @param X_world Output: transform from link i to world (one per link)
void forward_kinematics(const Articulation& model,
                        const VecXf& q,
                        std::vector<SpatialTransform>& X_J,
                        std::vector<SpatialTransform>& X_up,
                        std::vector<Transform>& X_world);

/// Inverse dynamics via Recursive Newton-Euler Algorithm (RNEA).
/// Computes generalized forces tau given q, qd, qdd and external forces.
///
/// @param model  Articulation model
/// @param q      Generalized positions
/// @param qd     Generalized velocities
/// @param qdd    Generalized accelerations
/// @param gravity Gravity vector (in world frame)
/// @param f_ext  External spatial forces per link (in link frame), or empty
/// @return tau   Generalized forces
VecXf inverse_dynamics(const Articulation& model,
                       const VecXf& q,
                       const VecXf& qd,
                       const VecXf& qdd,
                       const Vec3f& gravity,
                       const std::vector<SpatialVector>& f_ext = {});

/// Mass matrix via Composite Rigid Body Algorithm (CRBA).
/// Computes the joint-space inertia matrix H(q).
///
/// @param model  Articulation model
/// @param q      Generalized positions
/// @return H     Joint-space mass matrix (nv x nv, symmetric positive definite)
MatXf mass_matrix(const Articulation& model,
                  const VecXf& q);

/// Forward dynamics: compute joint accelerations from applied torques.
/// qdd = H(q)^{-1} * (tau - C(q, qd)) where C = RNEA(q, qd, 0)
///
/// @param model  Articulation model
/// @param q      Generalized positions
/// @param qd     Generalized velocities
/// @param tau    Applied joint torques
/// @param gravity Gravity vector
/// @param f_ext  External spatial forces per link (in link frame), or empty
/// @return qdd   Generalized accelerations
VecXf forward_dynamics(const Articulation& model,
                       const VecXf& q,
                       const VecXf& qd,
                       const VecXf& tau,
                       const Vec3f& gravity,
                       const std::vector<SpatialVector>& f_ext = {});

}  // namespace featherstone

}  // namespace novaphy
