// Copied from c_simulator (riptide_simulator) so the controller does not depend on
// the simulator. Keep it identical to the simulator's copy: test_dynamics_parity
// compares the two whenever c_simulator is installed.
#include "riptide_mpc/marine_dynamics.hpp"
#include <algorithm>
#include <cmath>
#include <stdexcept>

namespace riptide_mpc {
namespace {
template <typename Matrix> void positiveSemidefinite(const Matrix &m, const char *name) {
    if (!m.allFinite() || !m.isApprox(m.transpose(), 1e-10))
        throw std::invalid_argument(std::string(name) + " must be finite and symmetric");
    Eigen::SelfAdjointEigenSolver<Matrix> solver(m);
    if (solver.info() != Eigen::Success || solver.eigenvalues().minCoeff() < -1e-10)
        throw std::invalid_argument(std::string(name) + " must be positive semidefinite");
}
} // namespace
MarineDynamics::MarineDynamics() {
    configure(1., Eigen::Matrix3d::Identity(), Matrix6d::Zero());
}
void MarineDynamics::configure(double mass, const Eigen::Matrix3d &inertia, const Matrix6d &added) {
    if (!std::isfinite(mass) || mass <= 0)
        throw std::invalid_argument("Mass must be finite and positive");
    positiveSemidefinite(inertia, "Rigid inertia");
    Eigen::SelfAdjointEigenSolver<Eigen::Matrix3d> eig(inertia);
    if (eig.eigenvalues().minCoeff() <= 0 || eig.eigenvalues().maxCoeff() > eig.eigenvalues().sum() / 2. + 1e-9)
        throw std::invalid_argument("Rigid inertia must be positive definite and satisfy the "
                                    "principal-moment triangle inequalities");
    positiveSemidefinite(added, "Added mass");
    vehicle_mass_ = mass;
    rigid_body_mass_.setZero();
    rigid_body_mass_.topLeftCorner<3, 3>() = mass * Eigen::Matrix3d::Identity();
    rigid_body_mass_.bottomRightCorner<3, 3>() = inertia;
    added_mass_ = added;
    mass_ = rigid_body_mass_ + added;
    Eigen::LLT<Matrix6d> factorization(mass_);
    if (factorization.info() != Eigen::Success)
        throw std::invalid_argument("Total mass must be positive definite");
    inverse_mass_ = factorization.solve(Matrix6d::Identity());
}
void MarineDynamics::configureDamping(const Matrix6d &linear, const Vector6d &quadratic,
                                      const Eigen::Vector3d &center) {
    positiveSemidefinite(linear, "Linear damping");
    if (!quadratic.allFinite() || quadratic.minCoeff() < 0 || !center.allFinite())
        throw std::invalid_argument("Quadratic damping must be finite/nonnegative; "
                                    "damping center must be finite");
    linear_damping_ = linear;
    quadratic_damping_ = quadratic;
    damping_center_ = center;
}
void MarineDynamics::configureHydrostatics(double density, double volume, const Eigen::Vector3d &cob,
                                           const Eigen::Vector3d &radii, double gravity, double level) {
    if (!std::isfinite(density) || density <= 0 || !std::isfinite(volume) || volume < 0 || !std::isfinite(gravity) ||
        gravity <= 0 || !cob.allFinite() || !radii.allFinite() || radii.minCoeff() <= 0 || !std::isfinite(level))
        throw std::invalid_argument("Invalid hydrostatic properties");
    density_ = density;
    volume_ = volume;
    cob_ = cob;
    radii_ = radii;
    gravity_ = gravity;
    water_level_ = level;
}
Eigen::Matrix3d MarineDynamics::skew(const Eigen::Vector3d &v) {
    Eigen::Matrix3d s;
    s << 0, -v.z(), v.y(), v.z(), 0, -v.x(), -v.y(), v.x(), 0;
    return s;
}
Matrix6d MarineDynamics::coriolis(const Matrix6d &mass, const Vector6d &velocity) {
    // Momentum form, including full off-diagonal added-mass coupling (MSS m2c).
    const Vector6d momentum = mass * velocity;
    Matrix6d c = Matrix6d::Zero();
    c.topRightCorner<3, 3>() = -skew(momentum.head<3>());
    c.bottomLeftCorner<3, 3>() = -skew(momentum.head<3>());
    c.bottomRightCorner<3, 3>() = -skew(momentum.tail<3>());
    return c;
}
Vector6d MarineDynamics::acceleration(const Vector6d &v, const Vector6d &vr, const Vector6d &tau,
                                      const Vector6d &dc) const {
    // Absolute-velocity formulation: ONLY added mass multiplies current
    // acceleration. MRB*v_dot + CRB(v)*v + MA*(v_dot-c_dot) + CA(vr)*vr = tau.
    return inverse_mass_ *
           (tau - coriolis(rigid_body_mass_, v) * v - coriolis(added_mass_, vr) * vr + added_mass_ * dc);
}
Vector6d MarineDynamics::dampingWrench(const Vector6d &v) const {
    Matrix6d H = Matrix6d::Identity();
    H.topRightCorner<3, 3>() = -skew(damping_center_);
    const Vector6d local = H * v;
    // Map point velocity AND force back to COM. This preserves dissipativity even
    // during simultaneous translation and rotation about an offset drag centre.
    return -H.transpose() *
           (linear_damping_ * local + (quadratic_damping_.array() * local.array().abs() * local.array()).matrix());
}
double MarineDynamics::submergedFraction(const Eigen::Vector3d &p, const Eigen::Quaterniond &orientation,
                                         Eigen::Vector3d *center) const {
    const auto q = orientation.normalized();
    const Eigen::Vector3d vertical = q.conjugate() * Eigen::Vector3d::UnitZ();
    const double height = (radii_.array() * vertical.array()).matrix().norm();
    const double c = std::clamp((water_level_ - (p + q * cob_).z()) / height, -1., 1.);
    const double fraction = (2. + 3. * c - c * c * c) / 4.;
    if (center) {
        *center = cob_;
        if (fraction > 1e-10 && fraction < 1.) {
            const double mean = -3. * std::pow(1. - c * c, 2) / (16. * fraction);
            *center += (radii_.array().square() * vertical.array()).matrix() * (mean / height);
        }
    }
    return fraction;
}
Vector6d MarineDynamics::restoringWrench(const Eigen::Vector3d &p, const Eigen::Quaterniond &orientation) const {
    const auto q = orientation.normalized();
    Eigen::Vector3d center;
    const double buoyancy = density_ * gravity_ * volume_ * submergedFraction(p, q, &center);
    const Eigen::Vector3d force = q.conjugate() * Eigen::Vector3d(0, 0, buoyancy);
    Vector6d tau;
    tau.head<3>() = q.conjugate() * Eigen::Vector3d(0, 0, buoyancy - vehicle_mass_ * gravity_);
    tau.tail<3>() = center.cross(force);
    return tau;
}
void MarineDynamics::validateState(const State13d &x) {
    if (!x.allFinite() || x.segment<4>(3).norm() < 1e-10)
        throw std::invalid_argument("State must be finite with a nonzero quaternion");
}
State13d MarineDynamics::derivative(const State13d &x, const Vector6d &propulsion, const Eigen::Vector3d &water,
                                    const Eigen::Vector3d &waterDot) const {
    validateState(x);
    if (!propulsion.allFinite() || !water.allFinite() || !waterDot.allFinite())
        throw std::invalid_argument("Model inputs must be finite");
    const Eigen::Quaterniond q = Eigen::Quaterniond(x[3], x[4], x[5], x[6]).normalized();
    const Vector6d v = x.tail<6>();
    Vector6d c = Vector6d::Zero(), dc = Vector6d::Zero();
    c.head<3>() = q.conjugate() * water;
    dc.head<3>() = q.conjugate() * waterDot - v.tail<3>().cross(c.head<3>());
    const Vector6d relative = v - c;
    Eigen::Vector3d wetCenter;
    const double wet = submergedFraction(x.head<3>(), q, &wetCenter);
    // Uniform accelerating fluid has an undisturbed pressure gradient as well
    // as added-mass reaction (Froude-Krylov). Use inertial water acceleration,
    // never the rotating-coordinate dc term, for this applied pressure force.
    const Eigen::Vector3d pressureForce = density_ * volume_ * wet * (q.conjugate() * waterDot);
    Vector6d fluidPressure;
    fluidPressure << pressureForce, wetCenter.cross(pressureForce);
    const Vector6d wrench =
        propulsion + restoringWrench(x.head<3>(), q) + wet * dampingWrench(relative) + fluidPressure;
    State13d dx;
    dx.head<3>() = q * v.head<3>();
    const Eigen::Quaterniond dq = q * Eigen::Quaterniond(0, v[3], v[4], v[5]);
    dx.segment<4>(3) << dq.w() / 2, dq.x() / 2, dq.y() / 2, dq.z() / 2;
    dx.tail<6>() = acceleration(v, relative, wrench, dc);
    return dx;
}
State13d MarineDynamics::step(const State13d &x, const Vector6d &tau, double dt, const Eigen::Vector3d &water,
                              const Eigen::Vector3d &dw) const {
    if (!std::isfinite(dt) || dt <= 0 || dt > .1)
        throw std::invalid_argument("RK4 step must be in (0,0.1] seconds");
    const State13d k1 = derivative(x, tau, water, dw);
    const State13d k2 = derivative(x + dt * .5 * k1, tau, water + dt * .5 * dw, dw);
    const State13d k3 = derivative(x + dt * .5 * k2, tau, water + dt * .5 * dw, dw);
    const State13d k4 = derivative(x + dt * k3, tau, water + dt * dw, dw);
    State13d next = x + dt / 6 * (k1 + 2 * k2 + 2 * k3 + k4);
    validateState(next);
    next.segment<4>(3).normalize();
    return next;
}
} // namespace riptide_mpc
