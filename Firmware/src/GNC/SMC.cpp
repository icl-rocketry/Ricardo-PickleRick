// #include <Eigen/Dense>
// #include "SMC.h"
// #include<cmath>

// void LQR_SMC::setup(){
//     createTestK_p(); 
//     createTestK_i(); 
//     createTestK_d(); //cpp is a sequencial language

//     m_setpoint << 0.0, 0.0, 0.0, 0.0, 0.0, 0.0;

    
//     m_timestep = 0.01; 
//     m_previous_error << 0.0, 0.0, 0.0, 0.0, 0.0, 0.0;
//     m_integral_error_riemman << 0.0, 0.0, 0.0, 0.0, 0.0, 0.0;
//     m_integral_error_trapezoid << 0.0, 0.0, 0.0, 0.0, 0.0, 0.0;
//     m_previousSampleTime = millis();
// }

// void LQR_SMC::update(Eigen::Matrix<float,1, 6> currentPosition){
//     if (millis() - m_previousSampleTime >= m_actuationDelta) {
//         updateErrors(currentPosition); 
//         updateActuationValues(); 
//         m_previousSampleTime = millis();
//     }
// }

// void LQR_SMC::updateErrors(Eigen::Matrix<float,1, 6> currentPosition){

//     // Serial.println("Current position: " + String(currentPosition(0,3))); 

//     //Proportional Error = setpoint - inputMatrix; 
//     for (int i = 0; i < m_setpoint.cols(); i++) {
//         m_error (0,i) = m_setpoint(0,i) - currentPosition(0,i);
//         }
//     // Serial.println("Proportional Error: " + String(error(0,3))); 


//     //Integral Error = 
//         //Righthand Riemman Sum
//     for (int i = 0; i < m_setpoint.cols(); i++) {
//         m_integral_error_riemman(0,i) += m_error(0,i)*m_timestep ; 
//         }
//     // Serial.println("Int Riemman Error: " + String(integral_error_riemman(0,3))); 

//         //Trapezoid Rule
//     for (int i = 0; i < m_setpoint.cols(); i++) {
//         m_integral_error_trapezoid(0,i) += (m_error(0,i) + m_previous_error(0,i))*m_timestep*0.5 ; 
//         }
//     // Serial.println("Int Tra Error: " + String(integral_error_trapezoid(0,3))); 

//     //Derivative Error = 
//     for (int i = 0; i < m_setpoint.cols(); i++) {
//         m_derivative_error(0,i) = (m_error(0,i) - m_previous_error(0,i))/m_timestep ; 
//         }
//     // Serial.println("Derivative Error: " + String(derivative_error(0,3))); 

//     m_previous_error = m_error; 
// }

// void LQR_SMC::updateSMC(){

//     //define sliding surface
//     Eigen::Matrix<float, 1, 3> s = m_error.block<4, 6>(0, 0) + lambda_smc.cwiseProduct(m_error.block<1, 3>(0, 0));
//     Eigen::Matrix<float, 1, 3> acceleration;
//     SMC_x = -eta_x * tanh(sx / psi_x);
//     acceleration << eta_smc(0,0)*tanh(s(0,0)/psi_smc(0,0)), eta_smc(0,1)*tanh(s(0,1)/psi_smc(0,1)), eta_smc(0,2)*tanh(s(0,2)/psi_smc(0,2));

//     double mass = 1.4;
//     double T = mass * sqrt(acceleration(0,0)*acceleration(0,0) + acceleration(0,1)*acceleration(0,1) + (acceleration(0,2) + 9.81)*(acceleration(0,2) + 9.81));
//     double Tx = mass * acceleration(0,0);
//     double Ty = mass * acceleration(0,1);
//     double alpha = asin(-Tx/T);
//     double beta = asin(Ty/(T*cos(alpha)));
//     m_smc_actuation_values << T/2,T/2, alpha, beta;


// }

// void LQR_SMC::updateActuationValues(){

//     m_actuation_values = m_error * m_K_p + K_i * integral_error_riemman + m_smc_actuation_values;
//     // Serial.println("Actuation Values: " + String(actuation_values(0,0)) + " " + String(actuation_values(0,1)) + " " + String(actuation_values(0,2)) + " " + String(actuation_values(0,3)));
// }

// void LQR_SMC::createTestK_p() {
//     m_K_p << 0, 0, 0, 0,
//     0, 0, 0, 0,
//     0, 0, 0, 0,
//     0, 1, 0, 0,
//     -1, 0, 0, 0,
//     0, 0, 0, 0;
// }

// void LQR_SMC::createTestK_i() {
//     m_K_i << 0, 0, 0, 0,
//     0, 0, 0, 0,
//     0, 0, 0, 0,
//     0, 0, 0, 0,
//     0, 0, 0, 0,
//     0, 0, 0, 0;
// }

// void SMC::createTestLambda() {
//     lambda_smc << 0.1, 0.1, 0.1;
// }

// void SMC::createTestEta() {
//     eta_smc << 10, 10, 0.1;
// }

// void SMC::createTestPsi() {
//     psi_smc << 1, 1, 1;
// }

// void SMC::calibrate_impl(packetptr_t packetptr) { // saves SMC gains for all p, i & d matrices
//     SMCCalibrationPacket calibrate_comm(*packetptr);
//     std::vector<uint8_t> serializedData = packetptr->getBody();
//     //should be able to access k_11 k_12 from SMCcalibpacket
//     //save that into the kp gains matrix
//     std::string NVSName = "FRANCIS";
//     NVSStore _NVS(NVSName, NVSStore::calibrationType::Servo);
    
//     _NVS.saveBytes(serializedData);

//     // READ FUN

   
// };

// void SMC::check_gains() {
//     std::string NVSName = "FRANCIS";
//     NVSStore _NVS(NVSName, NVSStore::calibrationType::Servo);
//     //load gains matrix and print it all to see if the calib command can change the values
//     std::vector<uint8_t> calibSerialised = _NVS.loadBytes();
//     if(calibSerialised.size() == 0)
//     {
//         // setNormalState(0); // default is nominally closed
//         Serial.println("no data") ;// default NC
//         return;
//     }
//     Serial.println("SMC data:");
//     // setNormalState(calibpacket.normalState);
//    // Display the matrix
//     for (size_t j = 0; j < 4; ++j)
//     {
//         for (size_t i = 0; i < 6; ++i)
//         {
//             Serial.print(calibSerialised[j * 6 + i]);// j starts from 0, so first row is [0,x] so 0*6 = 6, plus the i can get the column number
//             Serial.print("  "); //space between te data
//         }
//         Serial.println();// back to the loop
//     }
// }

// void SMC::telemetry_impl(packetptr_t packetptr) {
//     SimpleCommandPacket packet(*packetptr);

// 	SMCTelemetryPacket telemetry;

// 	telemetry.header.type = 108;
// 	telemetry.header.source = packet.header.destination;
// 	telemetry.header.source_service = m_serviceID;
// 	telemetry.header.destination = packet.header.source;
// 	telemetry.header.destination_service = packet.header.source_service;
// 	telemetry.header.uid = packet.header.uid; 
// 	telemetry.pitch_angle = m_actuation_values(0,0);
// 	telemetry.roll_angle = m_actuation_values(0,1);
// 	telemetry.prop_0 = m_actuation_values(0,2);
// 	telemetry.prop_1 = m_actuation_values(0,3);

// 	m_networkmanager.sendPacket(telemetry);

// }
