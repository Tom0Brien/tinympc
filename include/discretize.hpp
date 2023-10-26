#ifndef TMPC_DISCRETIZE_HPP
#define TMPC_DISCRETIZE_HPP

#include <Eigen/Core>
#include <Eigen/Dense>
#include <unsupported/Eigen/src/MatrixFunctions/MatrixExponential.h>

namespace tinympc {

    template <typename Scalar, int n, int m, int p>
    struct DiscreteSystem {
        Eigen::Matrix<Scalar, n, n> Ad;
        Eigen::Matrix<Scalar, n, m> Bd;
        Eigen::Matrix<Scalar, p, n> Cd;
        Eigen::Matrix<Scalar, p, m> Dd;
    };

    template <typename Scalar, int n, int m, int p>
    DiscreteSystem<Scalar, n, m, p> discretize(const Eigen::Matrix<Scalar, n, n>& A,
                                               const Eigen::Matrix<Scalar, n, m>& B,
                                               const Eigen::Matrix<Scalar, p, n>& C,
                                               const Eigen::Matrix<Scalar, p, m>& D,
                                               Scalar Ts) {

        DiscreteSystem<Scalar, n, m, p> discrete_system;

        Eigen::Matrix<Scalar, n + m, n + m> P, eP;
        P << A, B, Eigen::Matrix<Scalar, m, n>::Zero(), Eigen::Matrix<Scalar, m, m>::Identity();

        eP = P.exp() * Ts;

        discrete_system.Ad = eP.block(0, 0, n, n);
        discrete_system.Bd = eP.block(0, n, n, m);
        discrete_system.Cd = C;  // C matrix remains unchanged
        discrete_system.Dd = D;  // D matrix remains unchanged

        return discrete_system;
    }

}  // namespace tinympc

#endif  // DISCRETIZE_HPP
