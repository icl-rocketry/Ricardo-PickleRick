#include <Eigen/Dense>
#include "PID.h"
Eigen::Matrix<float,1, 4> outputMat;

Eigen::Matrix<float,1, 4> PID::outputMatrix(Eigen::Matrix<float,1, 6> inputMatrix) {

    Eigen::Matrix<float, 1, 4> outputMat = Eigen::Matrix<float, 1, 4>::Zero();
    for (int j = 0; j < 4; ++j) {
            for (int i = 0; i < 6; ++i) {
                outputMat(0,j) += inputMatrix(0,i) * K_p(i,j);
            }
        }
    return outputMat;
}

void PID::createTestK_p() {
    K_p << 0, 0, 0, 0,
        0, 0, 0, 0,
        0, 0, 0, 0,
        1, 0, 0, 0,
        0, 1, 0, 0,
        0, 0, 0, 0;
}

void PID::Setup(){
    createTestK_p();
}