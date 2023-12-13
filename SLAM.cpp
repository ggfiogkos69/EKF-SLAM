#include <iostream>
//#include <Eigen/Dense>
#include "/Users/christos/Documents/headers/Eigen/Dense"

using namespace Eigen;


VectorXd kinematic_update(const VectorXd& pose, const VectorXd& velocity) 
{
    double dt = 0.1;  
    VectorXd new_pose(3);
    double v = velocity(0);  
    double omega = velocity(1);

    // Update the pose (x, y, theta)
    new_pose(0) = pose(0) + (v * std::cos(pose(2)) * dt) - (v * std::sin(pose(2)) * dt);  
    new_pose(1) = pose(1) + (v * std::sin(pose(2)) * dt) + (v * std::cos(pose(2)) * dt);  
    new_pose(2) = pose(2) + omega * dt;                                                   

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

MatrixXd covariance_update( MatrixXd& Sigma, const MatrixXd& Gt, int state_size)
{
    // Utilize block operations
    int N = state_size - 3;
    MatrixXd Sigma_vv = Sigma.topLeftCorner(3, 3);
    MatrixXd Sigma_mm = Sigma.bottomRightCorner(N, N);
    MatrixXd Sigma_vm = Sigma.topRightCorner(3, N);
    MatrixXd Sigma_mv = Sigma.bottomLeftCorner(N, 3);
  
    // Perform operations
    Eigen::MatrixXd Sigma_vv_new = Gt * Sigma_vv * Gt.transpose();
    Eigen::MatrixXd Sigma_mm_new = Sigma_mm;  
    Eigen::MatrixXd Sigma_vm_new = Gt * Sigma_vm;
    Eigen::MatrixXd Sigma_mv_new = Sigma_vm_new.transpose();  

    // Reconstruct Sigma
    Sigma.topLeftCorner(3, 3) = Sigma_vv_new;
    Sigma.bottomRightCorner(N, N) = Sigma_mm_new;
    Sigma.topRightCorner(3, N) = Sigma_vm_new;
    Sigma.bottomLeftCorner(N, 3) = Sigma_mv_new; 

    return Sigma;
}


void predictionStep(VectorXd& state_vector, MatrixXd& Sigma, const VectorXd& velocity, const Matrix3d& Q) 
{
    // Calculating jacobian of motion model
    MatrixXd Gt = motion_jacobian(state_vector.head(3), velocity); 

    // Noise Transformation into State Space
    MatrixXd Vt = noise_transformation(state_vector.head(3), velocity);
    MatrixXd Qt = Vt * Q * Vt.transpose();
    
    // State Prediction
    state_vector.head(3) = kinematic_update(state_vector.head(3), velocity);

    // Covariance Prediction
    Sigma = covariance_update(Sigma, Gt, state_vector.size());
}



void data_association()
{
    return;
}

void add_new_landmarks(VectorXd& state_vector, MatrixXd& Sigma, const VectorXd& measurements, int unmatched)
{
    double x = state_vector(0);
    double y = state_vector(1);
    double theta = state_vector(2);
    double range = measurements(0);
    double bearing = measurements(1);
    
    for(int i = 0; i < unmatched; ++i)
    {
        double x_land = x + range * cos(theta + bearing);
        double y_land = y + range * sin(theta + bearing);
        
        VectorXd new_state_vector(state_vector.size() + 2);
        new_state_vector << state_vector, x_land, y_land;

        MatrixXd Hu_inv(2,3);
            Hu_inv << 1, 0, -1 * range * sin(theta + bearing),
                      0, 1, range * cos(theta + bearing);

        MatrixXd H_inv = MatrixXd::Zero(2,state_vector.size());
        H_inv.block(0, 0, 2, 3) = Hu_inv;

        MatrixXd Hi_inv(2,2);
            Hi_inv << cos(theta + bearing), -1 * range * sin(theta + bearing),
                      sin(theta + bearing), range * cos(theta + bearing);

        Matrix2d Rt;                              
        Rt << 0.1, 0,
             0, 0.1;
        
        // Expand the covariance matrix
        MatrixXd new_Sigma = MatrixXd::Zero(Sigma.rows() + 2, Sigma.cols() + 2);
        new_Sigma.block(0, 0, Sigma.rows(), Sigma.cols()) = Sigma;

        new_Sigma.block(Sigma.rows(), 0, 2, Sigma.cols()) = H_inv * Sigma;
        //std::cout << "Sigma.transpose(): " << Sigma.rows() << "x" << Sigma.cols() << std::endl;
        new_Sigma.block(0, Sigma.cols(), Sigma.rows(), 2) = Sigma * H_inv.transpose();
        new_Sigma.block(Sigma.rows(), Sigma.cols(), 2, 2) = H_inv * Sigma * H_inv.transpose() + Hi_inv * Rt * Hi_inv.transpose();

        state_vector = new_state_vector;
        Sigma = new_Sigma;
    }
}

// UPDATE STEP
void updateStep(VectorXd& state_vector, MatrixXd& Sigma, const VectorXd& measurements, const MatrixXd& R) 
{
    double x = state_vector(0);
    double y = state_vector(1);
    double theta = state_vector(2);
    
    // Data association
    data_association();
    int measurements_num = 2;

    // Initializing new landmarks
    add_new_landmarks(state_vector, Sigma, measurements, 1);
    
    // Perception measurements
    double range = measurements(0);
    double bearing = measurements(1);
    MatrixXd zt(2,1);
        zt << range, 
              bearing;

    MatrixXd Ht = MatrixXd::Zero(2 * measurements_num, state_vector.size());              //CHANGE
    MatrixXd Dzt = MatrixXd::Zero(2 * measurements_num, 1);
    MatrixXd Rt = MatrixXd::Zero(2 * measurements_num, 2 * measurements_num);
    Rt.diagonal().array() = 0.1;

    int matched_landmarks = 1;
    for(int i=0; i<matched_landmarks; ++i)
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
            
            Dzt.block(2 * matched_landmarks, 0, 2, 1) = zt - zt_exp;
            
            MatrixXd Htu(2,3);
                Htu << - q_sqrt * dx, - q_sqrt * dy, 0,
                            dy, - dx, - q; 

            MatrixXd Htj(2,2);
                Htj << q_sqrt * dx, q_sqrt * dy,
                        - dy, dx;
            
            Ht.block(2 * matched_landmarks, 0, 2, 3) = Htu;
            Ht.block(2 * matched_landmarks, 2 * i + 3, 2, 2) = Htj;
    }

        // KALMAN GAIN
        MatrixXd Kt = Sigma * Ht.transpose() * ((Ht * Sigma * Ht.transpose()) + Rt).inverse();
        
        // FINAL STATE VectorXd
        state_vector = state_vector + Kt * Dzt; // or K* `Δzt

        // FINAL COV MATRIXXd
        Sigma = (MatrixXd::Identity(state_vector.size(), state_vector.size()) - Kt * Ht) * Sigma;
}
 
 
// random velocity "measurements"
VectorXd generateRandomMotion() {
    VectorXd velocity(2);
        velocity << 1.0 + 0.2 * rand() / RAND_MAX, 0.1 * (2.0 * rand() / RAND_MAX - 1.0);
    return velocity;
}

// random range - bearing "measurements"
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
   
    MatrixXd Sigma(3,3);                             // covariances
        Sigma << 0.5, 0, 0,
                 0, 0.1, 0,
                 0, 0, 0.1;   

    Matrix3d Q;                                      // model noise
        Q << 0.01, 0, 0,
             0, 0.01, 0,
             0, 0, 0.01;  
    
    Matrix2d R;                                      // sensor noise
        R << 0.1, 0,
             0, 0.1;
     
 int num_steps = 10;

    for (int step = 0; step < num_steps; ++step) {
        
        // Testing
        VectorXd velocity = generateRandomMotion();
        VectorXd measurements = generateRandomMeasurement();
        
        // Prediction step
        predictionStep(state_vector, Sigma, velocity, Q);

        // Update step
        updateStep(state_vector, Sigma, measurements, R);

        // Estimated State
        std::cout << "Step: " << step << ", Estimated State: " << state_vector.transpose() << std::endl;
    }

    return 0;
}
