#include <iostream>

void Perception_feed(){  //Landmark extraction, obtaining data about the surroundings
/*
1) Threshold for wrong measuremnets
2) 
*/
}

void Data_Association(){
    /*
    1) You might not re-observe landmarks every time step.
    2) You might observe something as being a landmark but fail to ever see it again.
    3) You might wrongly associate a landmark to a previously seen landmark.
    4) Nearest Neighbor using mahalanobis 
    5) JCBB as an improvement
    */
}

void Odometry_feed(){  //Initial Guess

//Get them at the same time via ROS?


}


void loop_closure(){}



int main(){
    std::cout<<"Building the blocks\n";
}



void initialization(){
    /*
    1) zero_mean_matrix
    2) zero_cov_matrix
    3) parameters (e.g F_x for 3d-> 2n+3 space)
    */

}

void EKF(auto mean_matrix, auto cov_matrix, auto measurements){
    
    /* PREDICTION STEP (MOTION)
    predictive_mean_matrix = (x',y',theta') = (x,y,theta) + translational/rotational velocity matrix
    
    MAP 3D TO 2N+3 DIMENSIONAL SPACE
    (x',y',theta', ...) = (x,y,theta, ...) + F_x^T * translational/rotational velocity matrix
    --------------------------------------------------------------------------------------------*/

    /* PREDITION STEP (COVARIANCE)
    Calculate Jacobian of g, G_t = (G_t^x 0 0 I)
    Calculate G_t^x = derivative(x',y',theta')
    predictive_cov_matrix = G_t * cov_matrix(t-1) * G_t^T + R_t
    --------------------------------------------------------------------------------------------*/

    /* CORRECTION STEP
    Calculate Kalman Gain
    Calculate H_t, Jacobian of the observation function
    Calculate the observation function
    Range-Bearing Observation
    If landmark has not be observed, initialize it
     
      

    --------------------------------------------------------------------------------------------*/



   
   
   
   
   
   
   
   
    /*
    1) Landmark extraction
    2) Data association
    3) Update the current state estimate using odometry data
    4) Update the estimated state from re-observing landmarks
    5) Add new landmarks to the current state
    6) Prediction and Correction
    */
}
