![image](https://github.com/Tom0Brien/tinympc/assets/41043317/b59edd8c-7c2a-4177-87f2-f8638d757d9e)

**tinympc** is a lightweight C++ library that implements general linear and non-linear model predictive control (MPC) algorithms using [Eigen3](https://eigen.tuxfamily.org/index.php?title=Main_Page), [osqp-eigen](https://robotology.github.io/osqp-eigen/index.html) and [NLopt](https://nlopt.readthedocs.io/en/latest/).

## Features

- Uses efficient sparse matrix computations via Eigen3.
- Allows users to define custom system dynamics, constraints, and cost functions.
- Robust error handling and solver feedback.
- Warm start capability for faster real-time optimization.

## Usage

After initializing the [MPC](https://github.com/Tom0Brien/tinympc/blob/main/include/mpc.hpp) object with system matrices, constraints, and desired weights, the MPC can be run in two main steps:

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

See examples folder.

## Contact

For any inquiries, bug reports, or feature requests, please open an issue on our GitHub repository.
