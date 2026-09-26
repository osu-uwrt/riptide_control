#include "riptide_mpc/box_qp.hpp"

#include <vector>

namespace riptide_mpc {
BoxQpResult solveBoxQp(const Eigen::MatrixXd &H, const Eigen::VectorXd &g, const Eigen::VectorXd &lb,
                       const Eigen::VectorXd &ub, Eigen::VectorXd &x, int max_iterations, double tolerance) {
    const Eigen::Index n = g.size();
    auto objective = [&](const Eigen::VectorXd &v) { return 0.5 * v.dot(H * v) + g.dot(v); };
    x = x.cwiseMax(lb).cwiseMin(ub);
    BoxQpResult result;
    std::vector<Eigen::Index> free;
    free.reserve(n);
    for (; result.iterations < max_iterations; ++result.iterations) {
        const Eigen::VectorXd gradient = H * x + g;
        // A bound is binding when it is (nearly) active and the gradient pushes into it.
        const double eps = 1e-9;
        free.clear();
        double stationarity = 0;
        for (Eigen::Index i = 0; i < n; ++i) {
            const bool at_lower = x[i] <= lb[i] + eps && gradient[i] > 0;
            const bool at_upper = x[i] >= ub[i] - eps && gradient[i] < 0;
            if (!at_lower && !at_upper) {
                free.push_back(i);
                stationarity = std::max(stationarity, std::abs(gradient[i]));
            }
        }
        if (free.empty() || stationarity < tolerance) {
            result.converged = true;
            break;
        }
        const Eigen::Index m = static_cast<Eigen::Index>(free.size());
        Eigen::MatrixXd Hff(m, m);
        Eigen::VectorXd gf(m);
        for (Eigen::Index a = 0; a < m; ++a) {
            gf[a] = gradient[free[a]];
            for (Eigen::Index b = 0; b < m; ++b)
                Hff(a, b) = H(free[a], free[b]);
        }
        const Eigen::VectorXd df = Hff.llt().solve(-gf);
        Eigen::VectorXd direction = Eigen::VectorXd::Zero(n);
        for (Eigen::Index a = 0; a < m; ++a)
            direction[free[a]] = df[a];

        const double f0 = objective(x);
        double alpha = 1.;
        Eigen::VectorXd candidate;
        for (int backtrack = 0; backtrack < 30; ++backtrack, alpha *= 0.5) {
            candidate = (x + alpha * direction).cwiseMax(lb).cwiseMin(ub);
            if (objective(candidate) <= f0 + 1e-4 * gradient.dot(candidate - x))
                break;
        }
        const double change = (candidate - x).lpNorm<Eigen::Infinity>();
        x = candidate;
        if (change < 1e-12) {
            result.converged = true;
            break;
        }
    }
    return result;
}
} // namespace riptide_mpc
