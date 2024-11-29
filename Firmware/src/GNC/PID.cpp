#include <Eigen/Dense>
#include "PID.h"

Eigen::Matrix<float,4, 6> PID::outputMatrix(Eigen::Matrix<float,4, 6> inputMatrix) {

    Eigen::Matrix<float, 4, 6> outputMat = Eigen::Matrix<float, 4, 1>::Zero();
    for (int j = 0; j < 4; ++j) {
            for (int i = 0; i < 6; ++i) {
                // outputMat(j, i) += inputMatrix(j, i) * K_pid(j, i);
            }
        }
    return outputMat;
};

void PID::createTestK_p() {
    K_p << 0, 0, 0, 0, 0, 0;
    0, 0, 0, 0, 0, 0;
    0, 0, 0, 0, 0, 0;
    0, 0, 0, 0, 0, 0;
}

void PID::save_pid_gains() {
    std::string NVSName = "FRANCIS";
    NVSStore _NVS(NVSName, NVSStore::calibrationType::Servo);
        //save to nvs store
    PIDcalibrationpacket PIDCalibration;
    std::vector<uint32_t> serializedData = PIDCalibraion.getBody();
        _NVS.saveBytes(serializedData);

// READ FUN

    std::vector<uint32_t> calibSerialised = _NVS.loadBytes();
    if(calibSerialised.size() == 0)
    {
        // setNormalState(0); // default is nominally closed
        Serial.println('no data') ;// default NC
        return;
    }
    Serial.println("PID data:");
    // setNormalState(calibpacket.normalState);
   // Display the matrix
for (size_t j = 0; j < 4; ++j)
{
    for (size_t i = 0; i < 6; ++i)
    {
        Serial.print(calibSerialised[j * 6 + i]);// j starts from 0, so first row is [0,x] so 0*6 = 6, plus the i can get the column number
        Serial.print("  "); //space between te data
    }
    Serial.println();// back to the loop
}
};