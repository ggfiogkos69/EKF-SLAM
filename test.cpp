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
Vector kinematic_update(const Vector& pose, const Vector& velocity) {
    double dt = 1.0;  
    Vector new_pose(3);
    double v = velocity(0);  
    double omega = velocity(1);

    // Update the pose
    new_pose(0) = pose(0) - (v * std::sin(pose(2)) / omega) + (v * std::sin(pose(2) + omega * dt) / omega);  // x
    new_pose(1) = pose(1) + (v * std::cos(pose(2)) / omega) - (v * std::cos(pose(2) + omega * dt) / omega);  // y
    new_pose(2) = pose(2) + omega * dt;                                                                      // theta

    return new_pose;
}

Matrix motion_jacobian(const Vector& pose, const Vector& velocity) {
    double dt = 1.0;  
    double v = velocity(0);  
    double omega = velocity(1);

    Matrix Gx = Matrix::Zero(3, 3);
    Gx(0,0) = 1.0; 
    Gx(0,2) = - (v * std::cos(pose(2)) / omega) + (v * std::cos(pose(2) + omega * dt) / omega);
    Gx(1,1) = 1.0;
    Gx(1,2) = (v * std::sin(pose(2)) / omega) - (v * std::sin(pose(2) + omega * dt) / omega);
    Gx(2,2) = 1.0;
    return Gx;
}



// PREDICTION STEP
void predictionStep(Vector& state_vector, Matrix& Sigma, const Vector& velocity, const Matrix& R) {
    // State Prediction
    state_vector.head(3) = kinematic_update(state_vector.head(3), velocity);

    // Covariance Prediction
    Matrix Gx = motion_jacobian(state_vector.head(3), velocity); 
    Eigen::Matrix<Eigen::MatrixXd, 2, 2> G;
    G(0,0) = Gx;
    G(0,1) = Matrix::Zero(state_vector.size(), state_vector.size());
    G(1,0) = Matrix::Zero(state_vector.size(), state_vector.size());
    G(1,1) = Matrix::Identity(state_vector.size(), state_vector.size());
    Sigma = G * Sigma * G.transpose() + R;
}

// UPDATE STEP
void updateStep(Vector& state_vector, Matrix& Sigma, const Vector& measurements, const Matrix& Q) {
    // MEASUREMENTS AND DATA ASSOCIATION FROM GRAPH_SLAM 
    double range = measurements(0);
    double bearing = measurements(1);
    // ADD STUFF
    Matrix H,z,h;

    // RANGE-BEARING OBSERVATION (eg 3 meters away, 60 degrees right)
    // δ, q, zi = h( μt)

        // GLOBAL AND LOCAL COORDINATES !!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!

    // KALMAN GAIN
        Matrix K = Sigma * H.transpose() * (H * Sigma * H.transpose() + Q).inverse();

    // FINAL STATE VECTOR
        state_vector = state_vector + K * (z - h);

    // FINAL COV MATRIX
        Sigma = (Matrix::Identity(state_vector.size(), state_vector.size()) - K * H) * Sigma;

 

}

  
int main() {
    // POSE, COV, NOISE INITIALIZATION
    int size = 3;
    state_vector = Vector::Zero(size);  
    Sigma = Matrix::Zero(size, size);   
    Q = Matrix::Zero(size, size);       
    R = Matrix::Zero(size, size);       

    int num_step = 69;

    for (int tt = 0; tt < num_step; ++tt) {
        
        // ROS ODOMETRY FEED
        Vector velocity;  

        // ROS PERCEPTION FEED
        Vector measurements;

        // Prediction step
        predictionStep(state_vector, Sigma, velocity, R);

        // Update step
        updateStep(state_vector, Sigma, measurements, Q);

    }

    return 0;
}
