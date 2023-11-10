#include <iostream>
//#include <Eigen/Dense>
#include "/Users/christos/Documents/headers/Eigen/Dense"

//VARIABLES
Eigen::MatrixXd state_matrix;
Eigen::MatrixXd cov_matrix;
Eigen::MatrixXd pred_state_matrix;
Eigen::MatrixXd pred_cov_matrix;

//CONSTANTS
Eigen::Matrix kinematic_model;
Eigen::Matrix dim_func;
Eigen::Matrix identity_matrix;


vo id state_prediction(){
    pred_state_matrix = state_matrix + dim_func.transpose() * kinematic_model; 
}

void measurement_prediction(){
    G = identity_matrix + dim_func.transpose() * diff_kinematic * dim_func;
    pred_cov_matrix = G * cov_matrix * G.transpose() + R
}

void obtained_measurement(){
    obtain to measurement
}

void data_association(){
    data association :-)
}

void update_step(){

}