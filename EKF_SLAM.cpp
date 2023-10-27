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
    */
}

void Odometry_feed(){  //Initial Guess

//Get them at the same time via ROS?


}
void EKF(){
    /*
    1) Landmark extraction
    2) Data association
    3) Update the current state estimate using odometry data
    4) Update the estimated state from re-observing landmarks
    5) Add new landmarks to the current state
    */
}

void loop_closure(){}



int main(){
    std::cout<<"Building the blocks\n";
}



