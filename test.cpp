#include <iostream>
//#include <Eigen/Dense>
#include "/Users/christos/Documents/headers/Eigen/Dense"


typedef Eigen::MatrixXd Matrix;
typedef Eigen::VectorXd Vector;

// Initialization
Vector state_vector;  // State vector [x, y, theta, landmark1_x, landmark1_y, landmark2_x, landmark2_y, ...]
Matrix Sigma;  // Covariance matrix

// Noise
Matrix Q;  
Matrix R;  

// Pose update - changing x,y,theta
Vector kinematic_update(const Vector& state, const Vector& velocity) {
    double dt = 1.0;  
    Vector new_state(3);
    double v = velocity(0);  
    double omega = velocity(1);

    // Update the pose
    new_state(0) = state(0) - (v * std::sin(state(2)) / omega) + (v * std::sin(state(2) + omega * dt) / omega);
    new_state(1) = state(1) + (v * std::cos(state(2)) / omega) - (v * std::cos(state(2) + omega * dt) / omega);
    new_state(2) = state(2) + omega * dt;

    return new_state;
}

// PREDICTION STEP
void predictionStep(Vector& state_vector, Matrix& Sigma, const Vector& velocity) {
    // State Prediction
    state_vector.head(3) = kinematic_update(state_vector.head(3), velocity);

    // Covariance Prediction
    //MATHHHHHHHHHH
    Matrix G = Matrix::Identity(state_vector.size(), state_vector.size());
    Sigma = G * Sigma * G.transpose() + R;
}

// UPDATE STEP
void updateStep(Vector& state_vector, Matrix& Sigma, const Vector& measurements) {
    // MEASUREMENTS AND DATA ASSOCIATION FROM GRAPH_SLAM
    // ADD STUFF
    Matrix H,z,h;
    
        // GLOBAL AND LOCAL COORDINATES !!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!

    // KALMAN GAIN
        Matrix K = Sigma * H.transpose() * (H * Sigma * H.transpose() + Q).inverse();

    // FINAL STATE VECTOR
        state_vector = state_vector + K * (z - h);

    // FINAL COV MATRIX
        Sigma = (Matrix::Identity(state_vector.size(), state_vector.size()) - K * H) * Sigma;

    // WE WILL SEE IF IT WILL RETURN state vector, Sigma

}


int main() {
    // POSE, COV, NOISE INITIALIZATION
    int num_step = 69;

    for (int tt = 0; tt < num_step; ++tt) {
        
        // ROS ODOMETRY FEED
        Vector velocity;  

        // ROS PERCEPTION FEED
        Vector measurements;

        // Prediction step
        predictionStep(state_vector, Sigma, velocity);

        // Update step
        updateStep(state_vector, Sigma, measurements);

    }

    return 0;
}
