#include <iostream>
//#include <Eigen/Dense>
#include "/Users/christos/Documents/headers/Eigen/Dense"

using namespace Eigen;



// Pose update - changing x,y,theta
VectorXd kinematic_update(const VectorXd& pose, const VectorXd& velocity) 
{
    double dt = 0.1;  
    VectorXd new_pose(3);
    double v = velocity(0);  
    double omega = velocity(1);

    // Update the pose
    new_pose(0) = pose(0) + (v * std::cos(pose(2)) * dt) - (v * std::sin(pose(2)) * dt);  // x
    new_pose(1) = pose(1) + (v * std::sin(pose(2)) * dt) + (v * std::cos(pose(2)) * dt);  // y
    new_pose(2) = pose(2) + omega * dt;                                                   // theta

    return new_pose;
}

MatrixXd motion_jacobian(const VectorXd& pose, const VectorXd& velocity) 
{
    double dt = 0.1;  
    double v = velocity(0);  
    double omega = velocity(1);
    double theta = pose(2);

    Eigen::Matrix3d Gx;
        Gx << 1, 0, v * sin(theta) * dt - v * cos(theta) * dt,
              0, 1, v * cos(theta) * dt - v * sin(theta) * dt,
              0, 0, dt;
    
    return Gx;
}

MatrixXd noise_transformation(const VectorXd& pose, const VectorXd& velocity)
{
    double dt = 0.1;  
    double v = velocity(0);  
    double omega = velocity(1);
    double theta = pose(2);

    Eigen::Matrix3d Vx;
        Vx << cos(theta) * dt, - sin(theta) * dt, 0,
              sin(theta) * dt, cos(theta) * dt, 0,
              0, 0, 1;

    return Vx;
}

// PREDICTION STEP
void predictionStep(VectorXd& state_vector, MatrixXd& Sigma, const VectorXd& velocity, const Matrix3d& Q) 
{
    // Calculating jacobian of motion model
    MatrixXd Gt = motion_jacobian(state_vector.head(3), velocity); 

    MatrixXd Vt = noise_transformation(state_vector.head(3), velocity);

    // Transformation into State Space
    MatrixXd Qt = Vt * Q * Vt.transpose();
    
    // State Prediction
    state_vector.head(3) = kinematic_update(state_vector.head(3), velocity);

   // Covariance Prediction
    int N = state_vector.size() - 3;
    MatrixXd Sigma_vv = Sigma.topLeftCorner(3, 3);
    MatrixXd Sigma_mm = Sigma.bottomRightCorner(2 * N, 2 * N);
    MatrixXd Sigma_vm = Sigma.topRightCorner(3, 2 * N);
    MatrixXd Sigma_mv = Sigma.bottomLeftCorner(2 * N, 3);
  
    Eigen::MatrixXd Sigma_vv_new = Gt * Sigma_vv * Gt.transpose();
    Eigen::MatrixXd Sigma_mm_new = Sigma_mm;  // Sigma_mm does not change in the prediction step
    Eigen::MatrixXd Sigma_vm_new = Gt * Sigma_vm;
    Eigen::MatrixXd Sigma_mv_new = Sigma_vm_new.transpose();  // Sigma_mv is the transpose of Sigma_vm

    // Combine the blocks back into the full covariance matrix
    Sigma.topLeftCorner(3, 3) = Sigma_vv_new;
    Sigma.bottomRightCorner(2 * N, 2 * N) = Sigma_mm_new;
    Sigma.topRightCorner(3, 2 * N) = Sigma_vm_new;
    Sigma.bottomLeftCorner(2 * N, 3) = Sigma_mv_new; 
}

/*

void data_association()
{
    return;
}

void add_landmarks()
{
    return;
}

// UPDATE STEP
void updateStep(VectorXd& state_vector, Matrix3d& Sigma, const VectorXd& measurements, const MatrixXd& R) 
{
    double x = state_vector(0);
    double y = state_vector(1);
    double theta = state_vector(2);
    
    // Data association
     data_association();
    int observed_num = 1;

    // Initializing new landmarks
    //add_landmarks();
    
    // Perception measurements
    double range = measurements(0);
    double bearing = measurements(1);
    MatrixXd zt(2,1);
        zt << range, 
              bearing;

    MatrixXd Ht = MatrixXd::Zeros(2 * observed_num, N);
    MatrixXd Dz = MatrixXd::Zeros(2 * observed_num, 1);
    MatrixXd Rt = MatrixXd::Zeros(2 * observed_num, 2 * observed_num);


    for(int i=0; i<observed_num; ++i)
    {
            double x_land = x + range * cos(bearing + theta);
            double y_land = y + range * sin(bearing + theta);
            double dx = x_land - x;
            double dy = y_land - y;

            MatrixXd d(2,1);
                d << dx,
                     dy;

            double q = (d.transpose() * d).value();
            double q_sqrt = sqrt(q);

            // EXPECTED OBSERVATION
            MatrixXd zt_exp(2,1);
                zt_exp << q_sqrt,
                     atan2(dy,dx);

            // JACOBIAN OF H
            MatrixXd Ht(2,3);
                Ht << - q_sqrt * dx, - q_sqrt * dy, 0,
                        dy, - dx, - q; 
            
            // KALMAN GAIN
            MatrixXd Kt = Sigma * Ht.transpose() * ((Ht * Sigma * Ht.transpose() + R)).inverse();

            // FINAL STATE VectorXd
            state_vector = state_vector + Kt * (zt - zt_exp); // or K* `Δzt

            // FINAL COV MATRIXXd
            Sigma = (MatrixXd::Identity(state_vector.size(), state_vector.size()) - Kt * Ht) * Sigma;
    }
}
 
 */

// Function to generate random motion command
VectorXd generateRandomMotion() {
    VectorXd velocity(2);
    velocity << 1.0 + 0.2 * rand() / RAND_MAX, 0.1 * (2.0 * rand() / RAND_MAX - 1.0);
    return velocity;
}

// Function to generate random measurement
VectorXd generateRandomMeasurement() {
    VectorXd measurement(2);
    measurement << 0.1 * rand() / RAND_MAX, 0.1 * (2.0 * rand() / RAND_MAX - 1.0);
    return measurement;
}


int main() 
{
    // POSE, COV, NOISE INITIALIZATION
    VectorXd state_vector(3);                        // x,y,theta,landmarks,colors(?)
        state_vector << 0, 0, 0;      
   
    Eigen::MatrixXd Sigma(3,3);                           // covariances
        Sigma << 0, 0, 0,
                 0, 0, 0,
                 0, 0, 0;   

    Eigen::Matrix3d Q;                               // model noise
        Q << 0.01, 0, 0,
             0, 0.01, 0,
             0, 0, 0.01;  
    
    Eigen::Matrix2d R;                               // sensor noise
        R << 0.1, 0,
             0, 0.1;
     
 int num_steps = 69;

    for (int step = 0; step < num_steps; ++step) {
        // Simulate random motion command
        VectorXd velocity = generateRandomMotion();
        //std::cout<<"WORKS1\n";

        // Simulate noisy sensor measurement
        VectorXd measurements = generateRandomMeasurement();
        //std::cout<<"WORKS2\n";
        
        // Prediction step
        predictionStep(state_vector, Sigma, velocity, Q);

        // Increase the size of the state vector by appending two new elements
        state_vector.conservativeResize(state_vector.size() + 2);
    
         // Set values for the new elements (assuming x and y coordinates of a new landmark)
        state_vector(state_vector.size() - 2) = 3.0; // x-coordinate of the new landmark
        state_vector(state_vector.size() - 1) = 4.0; // y-coordinate of the new landmark
        std::cout<<state_vector.size()<<std::endl;
        Sigma.conservativeResize(state_vector.size(), state_vector.size());

        // Update step (commented out for simplicity in this test)
        //updateStep(state_vector, Sigma, measurements, R);

        // Print the estimated state after each step
       // std::cout << "Step: " << step << ", Estimated State: " << state_vector.transpose() << std::endl;
    }

    return 0;
}
