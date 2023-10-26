#include <Eigen/Core>
#include <fstream>
#include <iostream>

#include "../include/discretize.hpp"
#include "../include/mpc.hpp"

int main() {

    // Dimensions
    const int n = 4;   // Number of states
    const int m = 1;   // Number of inputs
    const int N = 50;  // Prediction horizon

    // Define system matrices for the inverted pendulum on a cart
    Eigen::Matrix<double, n, n> A;
    Eigen::Matrix<double, n, m> B;
    Eigen::Matrix<double, 2, 4> C;
    Eigen::Matrix<double, 2, 1> D;
    A << 0, 1, 0, 0, 0, -0.1818, 2.673, 0, 0, 0, 0, 1, 0, -0.4545, 31.18, 0;
    B << 0, 1.818, 0, 4.545;
    C << 1, 0, 0, 0, 0, 0, 1, 0;
    D << 0, 0;

    // Discretize the system
    double Ts                               = 0.1;
    tinympc::DiscreteSystem discrete_system = tinympc::discretize(A, B, C, D, Ts);

    // Define weight matrices
    Eigen::Matrix<double, n, n> Q = Eigen::Matrix<double, n, n>::Identity();
    Eigen::Matrix<double, m, m> R = Eigen::Matrix<double, m, m>::Identity();

    // Define state and control bounds
    Eigen::Matrix<double, n, 1> x_min, x_max, x0, x_ref;
    Eigen::Matrix<double, m, 1> u_min, u_max;

    x_min << -10.0, -5.0, -M_PI / 4, -2.0;  // Just some arbitrary bounds for illustration
    x_max << 10.0, 5.0, M_PI / 4, 2.0;
    u_min << -10.0;  // Max force in the negative direction
    u_max << 10.0;   // Max force in the positive direction

    x0 << 0.0, 0.0, 0.5, 0.1;     // Initial state: pendulum slightly off vertical
    x_ref << 0.0, 0.0, 0.0, 0.0;  // Reference state: pendulum upright and cart stationary

    // Create an instance of the MPC class
    tinympc::MPC<double, n, m, N>
        mpc_controller(discrete_system.Ad, discrete_system.Bd, Q, R, x_min, x_max, u_min, u_max, x0, x_ref);

    // Control loop
    int num_iterations = 100;
    std::vector<Eigen::Matrix<double, n, 1>> states;
    std::vector<Eigen::Matrix<double, m, 1>> controls;
    for (int i = 0; i < num_iterations; ++i) {
        mpc_controller.compute_control();
        mpc_controller.propagate_system();
        states.push_back(mpc_controller.get_state());
        controls.push_back(mpc_controller.get_control());
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

    return 0;
}