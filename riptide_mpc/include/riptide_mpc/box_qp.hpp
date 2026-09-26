#pragma once

#include <Eigen/Dense>

namespace riptide_mpc {
struct BoxQpResult {
    int iterations = 0;
    bool converged = false;
};

// minimize 0.5 x'Hx + g'x  subject to  lb <= x <= ub,  H symmetric positive definite.
// Projected Newton (Bertsekas 1982): Newton step on the free set, projected Armijo
// line search. x holds the warm start on entry and the solution on exit.
BoxQpResult solveBoxQp(const Eigen::MatrixXd &H, const Eigen::VectorXd &g, const Eigen::VectorXd &lb,
                       const Eigen::VectorXd &ub, Eigen::VectorXd &x, int max_iterations = 50,
                       double tolerance = 1e-8);
} // namespace riptide_mpc
