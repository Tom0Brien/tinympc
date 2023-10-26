![tinympc](https://github.com/Tom0Brien/tinympc/assets/41043317/404607c2-4350-4e68-bada-0405234b7205)
**tinympc** is a lightweight C++ library that implements general linear and non-linear model predictive control (MPC) algorithms using [Eigen3](https://eigen.tuxfamily.org/index.php?title=Main_Page), [osqp-eigen](https://robotology.github.io/osqp-eigen/index.html) and [NLopt](https://nlopt.readthedocs.io/en/latest/).

## Features

- Uses efficient sparse matrix computations via Eigen3.
- Allows users to define custom system dynamics, constraints, and cost functions.
- Robust error handling and solver feedback.
- Warm start capability for faster real-time optimization.

## Usage

After initializing the [MPC](https://github.com/Tom0Brien/tinympc/blob/main/include/mpc.hpp) object with system matrices, constraints, and desired weights, the following functions are available:

| Function           | Description                                                                     |
| ------------------ | ------------------------------------------------------------------------------  |
| `compute_control()` | Solves the MPC optimization problem and computes the optimal control input for the current state.      |
| `get_control()`      | Get computed control input.                                    |
| `propagate_system()` | Propagates the system dynamics forward using the optimal control input obtained in the previous step.      |
| `get_states()`   | Get current states.                      |

## Dependencies

- [Eigen3](https://eigen.tuxfamily.org/index.php?title=Main_Page): Required for matrix computations.
- [OsqpEigen](https://github.com/robotology/osqp-eigen): Interface for the OSQP solver used to solve the quadratic program.
- [NLopt](https://nlopt.readthedocs.io/en/latest/). Non-linear optimization solver.

## Installation and Setup
  ```bash
  git clone https://github.com/Tom0Brien/tinympc.git && cd tinympc
  mkdir build && cd build
  cmake ..
  make
  sudo make install # copies files in the include folder to /usr/local/include*
  ```

## Examples

See examples folder for a number of examples.

Below is a simple example for a 2-state system that illustrates how to use the `tinympc::MPC` class:

---

### Example: Using the `tinympc::MPC` class for a 2-state system

1. **Include the necessary headers**:
```cpp
#include <tinympc/mpc.hpp>
```

2. **Define your system parameters**:
```cpp
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
```

3. **Instantiate the MPC class**:
```cpp
tinympc::MPC<double, 2, 1, N> mpc(A, B, Q, R, x_min, x_max, u_min, u_max, x0, x_ref);
```

4. **Compute and apply control for several steps**:
```cpp
for (int i = 0; i < 20; i++) {  // Simulate for 20 steps
    mpc.compute_control();
    mpc.propagate_system();
}
```


## Contact

For any inquiries, bug reports, or feature requests, please open an issue on our GitHub repository.
