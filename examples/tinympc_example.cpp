#include <Eigen/Core>
#include <iostream>

#include "../include/mpc.hpp"

int main() {

    // Define system dimensions
    constexpr int n = 12;  // Number of states
    constexpr int m = 4;   // Number of inputs
    constexpr int N = 20;  // MPC horizon

    // Define system matrices
    Eigen::Matrix<double, n, n> A;  // System dynamics matrix
    A << 1., 0., 0., 0., 0., 0., 0.1, 0., 0., 0., 0., 0., 0., 1., 0., 0., 0., 0., 0., 0.1, 0., 0., 0., 0., 0., 0., 1.,
        0., 0., 0., 0., 0., 0.1, 0., 0., 0., 0.0488, 0., 0., 1., 0., 0., 0.0016, 0., 0., 0.0992, 0., 0., 0., -0.0488,
        0., 0., 1., 0., 0., -0.0016, 0., 0., 0.0992, 0., 0., 0., 0., 0., 0., 1., 0., 0., 0., 0., 0., 0.0992, 0., 0., 0.,
        0., 0., 0., 1., 0., 0., 0., 0., 0., 0., 0., 0., 0., 0., 0., 0., 1., 0., 0., 0., 0., 0., 0., 0., 0., 0., 0., 0.,
        0., 1., 0., 0., 0., 0.9734, 0., 0., 0., 0., 0., 0.0488, 0., 0., 0.9846, 0., 0., 0., -0.9734, 0., 0., 0., 0., 0.,
        -0.0488, 0., 0., 0.9846, 0., 0., 0., 0., 0., 0., 0., 0., 0., 0., 0., 0., 0.9846;

    Eigen::Matrix<double, n, m> B;  // Input matrix
    B << 0., -0.0726, 0., 0.0726, -0.0726, 0., 0.0726, 0., -0.0152, 0.0152, -0.0152, 0.0152, -0., -0.0006, -0., 0.0006,
        0.0006, 0., -0.0006, 0.0000, 0.0106, 0.0106, 0.0106, 0.0106, 0, -1.4512, 0., 1.4512, -1.4512, 0., 1.4512, 0.,
        -0.3049, 0.3049, -0.3049, 0.3049, -0., -0.0236, 0., 0.0236, 0.0236, 0., -0.0236, 0., 0.2107, 0.2107, 0.2107,
        0.2107;

    Eigen::Matrix<double, n, n> Q;  // State cost matrix
    Q.diagonal() << 0, 0, 10., 10., 10., 10., 0, 0, 0, 5., 5., 5.;

    Eigen::Matrix<double, m, m> R;  // Input cost matrix
    R.diagonal() << 0.1, 0.1, 0.1, 0.1;

    // Constraints

    Eigen::Matrix<double, n, 1> x_min;  // Input lower bound
    x_min << -M_PI / 6, -M_PI / 6, -OsqpEigen::INFTY, -OsqpEigen::INFTY, -OsqpEigen::INFTY, -1., -OsqpEigen::INFTY,
        -OsqpEigen::INFTY, -OsqpEigen::INFTY, -OsqpEigen::INFTY, -OsqpEigen::INFTY, -OsqpEigen::INFTY;

    Eigen::Matrix<double, n, 1> x_max;  // Input upper bound
    x_max << M_PI / 6, M_PI / 6, OsqpEigen::INFTY, OsqpEigen::INFTY, OsqpEigen::INFTY, OsqpEigen::INFTY,
        OsqpEigen::INFTY, OsqpEigen::INFTY, OsqpEigen::INFTY, OsqpEigen::INFTY, OsqpEigen::INFTY, OsqpEigen::INFTY;

    Eigen::Matrix<double, m, 1> u_min;  // Input lower bound
    u_min << -0.9916, -0.9916, -0.9916, -0.9916;

    Eigen::Matrix<double, m, 1> u_max;  // Input upper bound
    u_max << 2.4084, 2.4084, 2.4084, 2.4084;

    Eigen::Matrix<double, n, 1> x0;  // Initial state
    x0 << 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0;

    Eigen::Matrix<double, n, 1> x_ref;  // Reference state
    x_ref << 0, 0, 1, 0, 0, 0, 0, 0, 0, 0, 0, 0;

    // Create MPC class instance
    tinympc::MPC<double, n, m, N> mpc(A, B, Q, R, x_min, x_max, u_min, u_max, x0, x_ref);

    // Solve MPC problem for 50 time steps
    int number_of_time_steps = 50;

    for (int i = 0; i < number_of_time_steps; i++) {
        // Solve MPC problem
        mpc.compute_control();

        // Propagate system
        mpc.propagate_system();
    }
}