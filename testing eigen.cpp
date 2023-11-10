#include <iostream>
#include "/Users/christos/Documents/headers/Eigen/Dense"


int main() {
    Eigen::Matrix<double,1,69> myMatrix = Eigen::Matrix<double,1,69>::Zero();
    int cnt = 3;
    int x=1,y=2,theta=60;
    myMatrix(0,0) = x;
    myMatrix(0,1) = y;
    myMatrix(0,2) = theta;
    myMatrix()

    std::cout << "Matrix:\n" << myMatrix << std::endl;

    return 0;
}

