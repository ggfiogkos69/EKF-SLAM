#include <iostream>
//#include <Eigen/Dense>
#include "/Users/christos/Documents/headers/Eigen/Dense"


typedef Eigen::MatrixXd Matrix;
typedef Eigen::VectorXd Vector;


// Pose update - changing x,y,theta
Vector kinematic_update(const Vector& pose, const Vector& velocity) 
{
    double dt = 1.0;  
    Vector new_pose(3);
    double v = velocity(0);  
    double omega = velocity(1);

    // Update the pose
    new_pose(0) = pose(0) + (v * std::cos(pose(2)) * dt) - (v * std::sin(pose(2)) * dt);  // x
    new_pose(1) = pose(1) + (v * std::sin(pose(2)) * dt) + (v * std::cos(pose(2)) * dt);  // y
    new_pose(2) = pose(2) + omega * dt;                                                   // theta

    return new_pose;
}

Matrix motion_jacobian(const Vector& pose, const Vector& velocity) 
{
    double dt = 1.0;  
    double v = velocity(0);  
    double omega = velocity(1);

    Matrix Gx = Matrix::Zero(3, 3);
    Gx(0,0) = 1.0; 
    Gx(0,2) = - (v * std::cos(pose(2)) / omega) + (v * std::cos(pose(2) + omega * dt) / omega);
    Gx(1,1) = 1.0;
    Gx(1,2) = (v * std::sin(pose(2)) / omega) - (v * std::sin(pose(2) + omega * dt) / omega);
    Gx(2,2) = dt;     //CHANGE
    return Gx;
}

Matrix noise_calibration(const Vector& pose, const Vector& velocity)
{
    double dt = 1.0;  
    double v = velocity(0);  
    double omega = velocity(1);
    double theta = pose(2);

    Matrix Vx = Matrix::Zero(3, 3);
    Vx(0,0) = cos(theta) * dt; 
    Vx(0,1) = - sin(theta) * dt;
    Vx(1,0) = sin(theta) * dt;
    Vx(1,1) = cos(theta) * dt;
    Vx(2,2) = 1;     
    return Vx;
}

// PREDICTION STEP
void predictionStep(Vector& state_vector, Matrix& Sigma, const Vector& velocity, const Matrix& Q) 
{
    // Calculating jacobian of motion model
    Matrix Gx = motion_jacobian(state_vector.head(3), velocity); 

    Matrix Vt = noise_calibration(state_vector.head(3), velocity);

    // Noise matrix calibration based on current state
    Matrix Qt = Vt * Q * Vt.transpose();
    
    // State Prediction
    state_vector.head(3) = kinematic_update(state_vector.head(3), velocity);

    // Covariance Prediction
    Eigen::Matrix<Eigen::MatrixXd, 2, 2> G;
    G(0,0) = Gx;
    G(0,1) = Matrix::Zero(state_vector.size(), state_vector.size());
    G(1,0) = Matrix::Zero(state_vector.size(), state_vector.size());
    G(1,1) = Matrix::Identity(state_vector.size(), state_vector.size());
    Sigma = (G * Sigma * G.transpose()) + Qt;
}



void data_association()
{
    return;
}

void add_landmarks()
{
    return;
}

// UPDATE STEP
void updateStep(Vector& state_vector, Matrix& Sigma, const Vector& measurements, const Matrix& Q) 
{
    double x = state_vector(0);
    double y = state_vector(1);
    
    // Data association
     data_association();
    
    // Initializing new landmarks
     add_landmarks();
    
    // Perception measurements
    double range = measurements(0);
    double bearing = measurements(1);

    // Loop update
    Vector H = Zeros(2i, N);
    Vector Dz = Zeros(2i,1);
    Vector R = Zeros(2i,2i);

    for(int i=0; i<observed_num; ++i)
    {
           float mjx = state_vector(0) + range * cos(bearing + state_vector(2));
           float mjy = state_vector(1) + range * sin(bearing + state_vector(2));
           float dx = mjx - mtx;
           float dy = mjy - mty;
            Vector d;
            d << dx, dy;
            Matrix d_trans = d.transpose();
            Matrix q = d * d_trans;
            float sqrt_q = sqrt(q);
            
            // EXPECTED OBSERVATION
            Matrix z;
            z << sqrt_q, atan2(dy,dx);

            // JACOBIAN OF H
            Matrix Ht;
            
            // KALMAN GAIN
            Matrix K = Sigma * H.transpose() * ((H * Sigma * H.transpose() + Q)).inverse();

            // FINAL STATE VECTOR
            state_vector = state_vector + K * (z - h); // or K* `Δzt

            // FINAL COV MATRIX
            Sigma = (Matrix::Identity(state_vector.size(), state_vector.size()) - K * H) * Sigma;
    }
}

  
int main() 
{
    // POSE, COV, NOISE INITIALIZATION
    Vector state_vector = Vector::Zero(3);      // x,y,theta,landmarks,colors(?)
    Matrix Sigma = Matrix::Zero(3, 3);          // covariances
    Matrix Q = 0.01 * Matrix::Identity(3, 3);   // model noise
    Matrix R = 0.1 * Matrix::Identity(2, 2);    // sensor noise
     

    int num_step = 69;

    for (int tt = 0; tt < num_step; ++tt) 
    {
        
        // ROS ODOMETRY FEED
        Vector velocity;  

        // ROS PERCEPTION FEED
        Vector measurements;

        // Prediction step
        predictionStep(state_vector, Sigma, velocity, Q);

        // Update step
        updateStep(state_vector, Sigma, measurements, Q);
    }

    return 0;
}
