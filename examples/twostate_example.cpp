#include <Eigen/Core>
#include <fstream>
#include <iostream>

#include "../include/mpc.hpp"

int main() {
    // Dimensions
    const int n = 2;   // Number of states
    const int m = 1;   // Number of inputs
    const int N = 50;  // Prediction horizon

    // Define system matrices for the 2-state system
    double Ts = 0.1;
    Eigen::Matrix<double, n, n> A;
    A << 1, Ts, Ts, 1;

    Eigen::Matrix<double, n, m> B;
    B << 0, Ts;

    // Define cost matrices
    Eigen::Matrix<double, n, n> Q = 10 * Eigen::Matrix<double, n, n>::Identity();
    Eigen::Matrix<double, m, m> R = Eigen::Matrix<double, m, m>::Identity();

    // Define state and input bounds
    Eigen::Matrix<double, n, 1> x_min, x_max;
    x_min << -10, -10;
    x_max << 10, 10;
    Eigen::Matrix<double, m, 1> u_min, u_max;
    u_min << -6;
    u_max << 6;

    // Define initial state and reference
    Eigen::Matrix<double, n, 1> x0, x_ref;
    x0 << 0.1, 0.1;
    x_ref << 1, 0;

    tinympc::MPC<double, 2, 1, N> mpc(A, B, Q, R, x_min, x_max, u_min, u_max, x0, x_ref);

    std::vector<Eigen::Matrix<double, n, 1>> states;
    std::vector<Eigen::Matrix<double, m, 1>> controls;
    // Simulate for 20 steps
    for (int i = 0; i < 1000; i++) {
        mpc.compute_control();
        Eigen::Matrix<double, 1, 1> u = mpc.get_control();
        mpc.propagate_system();
        states.push_back(mpc.get_state());
        controls.push_back(u);
    }

    // Save state and control to file
    std::ofstream file("history.csv");
    for (size_t i = 0; i < states.size(); ++i) {
        for (int j = 0; j < n; ++j) {
            file << states[i](j) << ",";
        }
        for (int j = 0; j < m; ++j) {
            file << controls[i](j) << (j < m - 1 ? "," : "\n");
        }
    }
    file.close();
}