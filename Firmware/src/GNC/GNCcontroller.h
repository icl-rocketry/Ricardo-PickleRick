#pragma once

#include <Eigen/Dense>
#include "librrc/Remote/nrcremotecontrollerbase.h"
#include <librnp/rnp_networkmanager.h>

#include <librrc/Helpers/nvsstore.h>
#include "GNC/PIDCalibrationPacket.h"
#include "GNC/PIDTelemetryPacket.h"
#include "Config/services_config.h"
#include <Arduino.h>
#include "GNC/PID.h"

class GNCcontroller {
    public: 
        GNCcontroller(PID& pid1 , PID& pid2) :
            pid1(pid1), 
            pid2(pid2) {};
        void setup(); 
        void update(Eigen::Matrix<float,1, 6> currentInput); 

        void first_PID_caculation_matrix(); 
        void first_PID_caculation_PID(); 
        void angle_error_to_input_second();
        void second_PID_caculation_matrix(); 
        void second_PID_caculation_PID(); 

        Eigen::Matrix<float,1, 4> getOutput(); 
        void sendActuationCommands(); 

        void create_input_first(); 
        void create_angle_error(); 
        void create_input_second(); 
        void create_output_second(); 
        // void create_m_first();
        // void create_m_second();
        void create_m_first_PID_config();
        void create_m_second_PID_config();

    private: 
        Eigen::Matrix<float,1, 6> input_first;
        Eigen::Matrix<float,1, 4> angle_error;
        Eigen::Matrix<float,1, 6> input_second; 
        Eigen::Matrix<float,1, 4> output_second;
        // Eigen::Matrix<float,6, 4> m_first; //sumulation
        // Eigen::Matrix<float,6, 4> m_second; //sumulation
        Eigen::Matrix<float,6, 4> m_first_PID_config; //actual
        Eigen::Matrix<float,6, 4> m_second_PID_config; //actual

        Eigen::Matrix<float,1, 6> setpoint_first; 
        Eigen::Matrix<float,1, 6> setpoint_second; 

        PID& pid1; 
        PID& pid2; 
}; 