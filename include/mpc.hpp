#ifndef TMPC_MPC_HPP
#define TMPC_MPC_HPP

#include <Eigen/Core>
#include <Eigen/Dense>
#include <OsqpEigen/OsqpEigen.h>

/** \file mpc.hpp
 * @brief Contains linear MPC class
 */
namespace tinympc {

    /** @brief MPC class
     */
    template <typename Scalar, int n, int m, int N>
    class MPC {
    public:
        // Constructor
        MPC(const Eigen::Matrix<Scalar, n, n>& A_in,
            const Eigen::Matrix<Scalar, n, m>& B_in,
            const Eigen::Matrix<Scalar, n, n>& Q_in,
            const Eigen::Matrix<Scalar, m, m>& R_in,
            const Eigen::Matrix<Scalar, n, 1>& x_min_in,
            const Eigen::Matrix<Scalar, n, 1>& x_max_in,
            const Eigen::Matrix<Scalar, m, 1>& u_min_in,
            const Eigen::Matrix<Scalar, m, 1>& u_max_in,
            const Eigen::Matrix<Scalar, n, 1>& x0_in,
            const Eigen::Matrix<Scalar, n, 1>& x_ref_in)
            : A(A_in)
            , B(B_in)
            , Q(Q_in)
            , R(R_in)
            , x_min(x_min_in)
            , x_max(x_max_in)
            , u_min(u_min_in)
            , u_max(u_max_in)
            , x0(x0_in)
            , x_ref(x_ref_in) {
            compute_hessian();
            compute_gradient();
            compute_linear_constraints();
            compute_constraint_vectors(x0_in);
            solver.settings()->setVerbosity(false);
            solver.settings()->setWarmStart(true);
            solver.data()->setNumberOfVariables(n * (N + 1) + m * N);
            solver.data()->setNumberOfConstraints(n * (N + 1) + n * (N + 1) + m * N);
            solver.data()->setHessianMatrix(hessian);
            solver.data()->setGradient(gradient);
            solver.data()->setLinearConstraintsMatrix(linear_constraints);
            solver.data()->setLowerBound(lower_bound);
            solver.data()->setUpperBound(upper_bound);
            solver.initSolver();
        }

        // Run MPC
        void compute_control() {
            // Solve the QP problem
            if (solver.solveProblem() != OsqpEigen::ErrorExitFlag::NoError) {
                std::cout << "The solver failed to find a solution." << std::endl;
            }

            // Get the controller input
            QPSolution = solver.getSolution();
            u          = QPSolution.block(n * (N + 1), 0, m, 1);
        }

        void propagate_system() {
            // Propagate the model
            x0 = A * x0 + B * u;

            // Update the constraint bound
            update_constraint_vectors(x0);
            if (!solver.updateBounds(lower_bound, upper_bound)) {
                std::cout << "The solver failed to update the bounds." << std::endl;
            }
        }

        // Get the controller input
        Eigen::Matrix<Scalar, m, 1> get_control() {
            return u;
        }

        // Get the state
        Eigen::Matrix<Scalar, n, 1> get_state() {
            return x0;
        }

        // Set the state
        void set_state(const Eigen::Matrix<Scalar, n, 1>& x0_in) {
            x0 = x0_in;
        }

    private:
        void compute_hessian() {
            hessian.resize(n * (N + 1) + m * N, n * (N + 1) + m * N);

            for (int i = 0; i < n * (N + 1) + m * N; i++) {
                if (i < n * (N + 1)) {
                    int posQ     = i % n;
                    Scalar value = Q.diagonal()[posQ];
                    if (value != static_cast<Scalar>(0))
                        hessian.insert(i, i) = value;
                }
                else {
                    int posR     = i % m;
                    Scalar value = R.diagonal()[posR];
                    if (value != static_cast<Scalar>(0))
                        hessian.insert(i, i) = value;
                }
            }
        }

        void compute_gradient() {
            Eigen::Matrix<Scalar, n, 1> Qx_ref = Q * (-x_ref);
            for (int i = 0; i < n * (N + 1); i++) {
                int posQ       = i % n;
                Scalar value   = Qx_ref(posQ, 0);
                gradient(i, 0) = value;
            }
        }

        void compute_linear_constraints() {
            linear_constraints.resize(n * (N + 1) + n * (N + 1) + m * N, n * (N + 1) + m * N);

            // Populate linear constraint matrix
            for (int i = 0; i < n * (N + 1); i++) {
                linear_constraints.insert(i, i) = static_cast<Scalar>(-1);
            }

            for (int i = 0; i < N; i++)
                for (int j = 0; j < n; j++)
                    for (int k = 0; k < n; k++) {
                        Scalar value = A(j, k);
                        if (value != static_cast<Scalar>(0)) {
                            linear_constraints.insert(n * (i + 1) + j, n * i + k) = value;
                        }
                    }

            for (int i = 0; i < N; i++)
                for (int j = 0; j < n; j++)
                    for (int k = 0; k < m; k++) {
                        Scalar value = B(j, k);
                        if (value != static_cast<Scalar>(0)) {
                            linear_constraints.insert(n * (i + 1) + j, m * i + k + n * (N + 1)) = value;
                        }
                    }

            for (int i = 0; i < n * (N + 1) + m * N; i++) {
                linear_constraints.insert(i + (N + 1) * n, i) = static_cast<Scalar>(1);
            }
        }

        void compute_constraint_vectors(const Eigen::Matrix<Scalar, n, 1>& x0) {
            // Evaluate the lower and the upper inequality vectors
            Eigen::Matrix<Scalar, Eigen::Dynamic, 1> lower_inequality =
                Eigen::Matrix<Scalar, Eigen::Dynamic, 1>::Zero(n * (N + 1) + m * N, 1);
            Eigen::Matrix<Scalar, Eigen::Dynamic, 1> upper_inequality =
                Eigen::Matrix<Scalar, Eigen::Dynamic, 1>::Zero(n * (N + 1) + m * N, 1);
            for (int i = 0; i < N + 1; i++) {
                lower_inequality.block(n * i, 0, n, 1) = x_min;
                upper_inequality.block(n * i, 0, n, 1) = x_max;
            }
            for (int i = 0; i < N; i++) {
                lower_inequality.block(m * i + n * (N + 1), 0, m, 1) = u_min;
                upper_inequality.block(m * i + n * (N + 1), 0, m, 1) = u_max;
            }

            // Evaluate the lower and the upper equality vectors
            Eigen::Matrix<Scalar, Eigen::Dynamic, 1> lower_equality =
                Eigen::Matrix<Scalar, Eigen::Dynamic, 1>::Zero(n * (N + 1), 1);
            Eigen::Matrix<Scalar, Eigen::Dynamic, 1> upper_equality;
            lower_equality.block(0, 0, n, 1) = -x0;
            upper_equality                   = lower_equality;

            // Merge inequality and equality vectors
            lower_bound << lower_equality, lower_inequality;
            upper_bound << upper_equality, upper_inequality;
        }

        void update_constraint_vectors(const Eigen::Matrix<Scalar, n, 1>& x0) {
            lower_bound.block(0, 0, n, 1) = -x0;
            upper_bound.block(0, 0, n, 1) = -x0;
        }


        /// @brief State dynamics matrix
        Eigen::Matrix<Scalar, n, n> A = Eigen::Matrix<Scalar, n, n>::Zero();

        /// @brief Input matrix
        Eigen::Matrix<Scalar, n, m> B = Eigen::Matrix<Scalar, n, m>::Zero();

        /// @brief State cost matrix
        Eigen::Matrix<Scalar, n, n> Q = Eigen::Matrix<Scalar, n, n>::Identity();

        /// @brief Input cost matrix
        Eigen::Matrix<Scalar, m, m> R = Eigen::Matrix<Scalar, m, m>::Identity();

        /// @brief State upper bound
        Eigen::Matrix<Scalar, n, 1> x_min = Eigen::Matrix<Scalar, n, 1>::Zero();

        /// @brief State lower bound
        Eigen::Matrix<Scalar, n, 1> x_max = Eigen::Matrix<Scalar, n, 1>::Zero();

        /// @brief Input upper bound
        Eigen::Matrix<Scalar, m, 1> u_min = Eigen::Matrix<Scalar, m, 1>::Zero();

        /// @brief Input lower bound
        Eigen::Matrix<Scalar, m, 1> u_max = Eigen::Matrix<Scalar, m, 1>::Zero();

        /// @brief State initial condition
        Eigen::Matrix<Scalar, n, 1> x0 = Eigen::Matrix<Scalar, n, 1>::Zero();

        /// @brief State reference
        Eigen::Matrix<Scalar, n, 1> x_ref = Eigen::Matrix<Scalar, n, 1>::Zero();

        /// @brief Hessian
        Eigen::SparseMatrix<Scalar> hessian;

        /// @brief Gradient
        Eigen::Matrix<Scalar, n*(N + 1) + m * N, 1> gradient = Eigen::Matrix<Scalar, n*(N + 1) + m * N, 1>::Zero();

        /// @brief Linear constraint matrix
        Eigen::SparseMatrix<Scalar> linear_constraints;

        /// @brief Lower bound vector
        Eigen::Matrix<Scalar, 2 * n*(N + 1) + m * N, 1> lower_bound =
            Eigen::Matrix<Scalar, Eigen::Dynamic, 1>::Zero(2 * n * (N + 1) + m * N, 1);

        /// @brief Upper bound vector
        Eigen::Matrix<Scalar, 2 * n*(N + 1) + m * N, 1> upper_bound =
            Eigen::Matrix<Scalar, Eigen::Dynamic, 1>::Zero(2 * n * (N + 1) + m * N, 1);

        /// @brief OsqpEigen Solver
        OsqpEigen::Solver solver;

        /// @brief Solver solution
        Eigen::VectorXd QPSolution;

        /// @brief Control input
        Eigen::Matrix<Scalar, m, 1> u = Eigen::Matrix<Scalar, m, 1>::Zero();
    };

}  // namespace tinympc

#endif
