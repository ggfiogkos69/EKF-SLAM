#include <iostream>
//#include <Eigen/Dense>
#include "/Users/christos/Documents/headers/Eigen/Dense"

using namespace Eigen;

// State transition function for a 2D system: x' = Ax + Bu + process_noise
MatrixXd stateTransition(const MatrixXd& A, const MatrixXd& B, const VectorXd& x, const VectorXd& u, const MatrixXd& process_noise) {
    return A * x + B * u + process_noise;
}

// Observation function for a 2D system: z = Cx + measurement_noise
VectorXd observationFunction(const MatrixXd& C, const VectorXd& x, const MatrixXd& measurement_noise) {
    return C * x + measurement_noise;
}

int main() {
    // Define the state dimension
    const int stateDim = 2;

    // Define the process and measurement noise covariance matrices
    const double processNoiseVar = 0.01;
    const double measurementNoiseVar = 0.1;

    MatrixXd Q = MatrixXd::Identity(stateDim, stateDim) * processNoiseVar;
    MatrixXd R = MatrixXd::Identity(stateDim, stateDim) * measurementNoiseVar;

    // Initial state estimate and covariance
    VectorXd x_hat(stateDim);
    x_hat << 0.0, 0.0; // Initial guess
    MatrixXd P(stateDim, stateDim);
    P << 1.0, 0.0,
         0.0, 1.0; // Initial guess covariance

    // System dynamics matrices
    MatrixXd A(stateDim, stateDim);
    A << 1.0, 1.0,
         0.0, 1.0;

    MatrixXd B(stateDim, 1);
    B << 0.5,
         1.0;

    // Observation matrix
    MatrixXd C = MatrixXd::Identity(stateDim, stateDim);

    // Simulation parameters
    const int numSteps = 100;
    const double dt = 0.1; // Time step

    for (int i = 0; i < numSteps; ++i) {
        // True state and input (for simulation purposes)
        VectorXd trueState = VectorXd::Zero(stateDim);
        trueState(0) = 2.0 * std::sin(0.1 * i); // Example function
        double input = 1.0;

        // Simulate the true system
        VectorXd processNoise = std::sqrt(processNoiseVar) * MatrixXd::Random(stateDim, 1);
        VectorXd newState = stateTransition(A, B, trueState, VectorXd::Constant(1, input), processNoise);

        // Simulate measurement with noise
        VectorXd measurementNoise = std::sqrt(measurementNoiseVar) * MatrixXd::Random(stateDim, 1);
        VectorXd measurement = observationFunction(C, newState, measurementNoise);

        // Extended Kalman Filter prediction step
        VectorXd predictedState = stateTransition(A, B, x_hat, VectorXd::Constant(1, input), MatrixXd::Zero(stateDim, 1));
        MatrixXd A_tilde = MatrixXd::Identity(stateDim, stateDim) + dt * A; // Jacobian of the state transition function

        P = A_tilde * P * A_tilde.transpose() + Q;

        // Extended Kalman Filter update step
        VectorXd y = measurement - observationFunction(C, predictedState, MatrixXd::Zero(stateDim, 1)); // Assuming zero mean for noise
        MatrixXd H = C; // Jacobian of the observation function

        MatrixXd S = H * P * H.transpose() + R;
        MatrixXd K = P * H.transpose() * S.inverse();

        x_hat = predictedState + K * y;
        P = (MatrixXd::Identity(stateDim, stateDim) - K * H) * P;

        // Print the estimated state
        std::cout << "Estimated State at Step " << i << ": " << x_hat.transpose() << std::endl;
    }

    return 0;
}
