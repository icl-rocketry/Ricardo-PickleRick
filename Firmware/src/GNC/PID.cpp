#include "PID.h"

void PID::setup(Eigen::Matrix<float,1, 6> m_personal_setpoint){
    createTestK_p(); 
    createTestK_i(); 
    createTestK_d(); 

    m_setpoint = m_personal_setpoint;
    
    m_timestep = 0.001; 
    m_previous_error << 0.0, 0.0, 0.0, 0.0, 0.0, 0.0;
    m_integral_error_riemman << 0.0, 0.0, 0.0, 0.0, 0.0, 0.0;
    m_integral_error_trapezoid << 0.0, 0.0, 0.0, 0.0, 0.0, 0.0;
    m_previousSampleTime = millis();
}

void PID::update(Eigen::Matrix<float,1, 6> currentPosition){
    if (millis() - m_previousSampleTime >= m_timestep*1000) {
        updateErrors(currentPosition); 
        updateOutputValues(); 
        m_previousSampleTime = millis();
    }
}

void PID::reset() {
    m_error << 0.0, 0.0, 0.0, 0.0, 0.0, 0.0;
    m_integral_error_riemman << 0.0, 0.0, 0.0, 0.0, 0.0, 0.0;
    m_integral_error_trapezoid << 0.0, 0.0, 0.0, 0.0, 0.0, 0.0;
    m_previous_error << 0.0, 0.0, 0.0, 0.0, 0.0, 0.0;
    m_derivative_error << 0.0, 0.0, 0.0, 0.0, 0.0, 0.0;
    m_sum_error << 0.0, 0.0, 0.0, 0.0, 0.0, 0.0;
    m_output_values << 0.0, 0.0, 0.0, 0.0;
    m_previousSampleTime = millis();
}
void PID::updateErrors(Eigen::Matrix<float,1, 6> currentPosition){

    // Serial.println("Current position: " + String(currentPosition(0,3))); 

    //Proportional Error = setpoint - inputMatrix; 
    for (int i = 0; i < m_setpoint.cols(); i++) {
        m_error (0,i) = m_setpoint(0,i) - currentPosition(0,i);
        }
    // Serial.println("Proportional Error: " + String(error(0,3))); 


    //Integral Error = 
        //Righthand Riemman Sum
    for (int i = 0; i < m_setpoint.cols(); i++) {
        m_integral_error_riemman(0,i) += m_error(0,i)*m_timestep ; 
        }
    // Serial.println("Int Riemman Error: " + String(integral_error_riemman(0,3))); 

        //Trapezoid Rule
    for (int i = 0; i < m_setpoint.cols(); i++) {
        m_integral_error_trapezoid(0,i) += (m_error(0,i) + m_previous_error(0,i))*m_timestep*0.5 ; 
        }
    // Serial.println("Int Tra Error: " + String(integral_error_trapezoid(0,3))); 

    //Derivative Error = 
    for (int i = 0; i < m_setpoint.cols(); i++) {
        m_derivative_error(0,i) = (m_error(0,i) - m_previous_error(0,i))/m_timestep ; 
        }
    // Serial.println("Derivative Error: " + String(derivative_error(0,3))); 

    for (int i = 0; i < m_setpoint.cols(); i++) {
        // 加权：
        // m_sum_error(0,i) = (m_error(0,i)*1 + m_integral_error_riemman(0,i)*1 +  m_integral_error_trapezoid(0,i)*1 + m_derivative_error(0,i)*1) ; 
        // 现在只用一个prop error：
        m_sum_error(0,i) = (m_error(0,i)*1 + m_integral_error_riemman(0,i)*0 +  m_integral_error_trapezoid(0,i)*0 + m_derivative_error(0,i)*0) ; 
        }

    m_previous_error = m_error; //only m_error for now
}

void PID::updateOutputValues(){
    m_output_values = m_error * m_K_p; // + K_i * integral_error_riemman + K_d * derivative_error; 
    // Serial.println("Actuation Values: " + String(actuation_values(0,0)) + " " + String(actuation_values(0,1)) + " " + String(actuation_values(0,2)) + " " + String(actuation_values(0,3)));
}

Eigen::Matrix<float,1, 4> PID::getOutputValues() {
    return m_output_values;
}

void PID::createTestK_p() {
    m_K_p << 0, 0, 0, 0,
    0, 0, 0, 0,
    0, 0, 0, 0,
    -1, 0, 0, 0,
    0, -1, 0, 0,
    0, 0, 0, 0;
}

void PID::createTestK_i() {
    m_K_i << 0, 0, 0, 0,
    0, 0, 0, 0,
    0, 0, 0, 0,
    0, 0, 0, 0,
    0, 0, 0, 0,
    0, 0, 0, 0;
}

void PID::createTestK_d() {
    m_K_d << 0, 0, 0, 0,
    0, 0, 0, 0,
    0, 0, 0, 0,
    0, 0, 0, 0,
    0, 0, 0, 0,
    0, 0, 0, 0;
}

void PID::calibrate_impl(packetptr_t packetptr) { // saves pid gains for all p, i & d matrices
    PIDCalibrationPacket calibrate_comm(*packetptr);
    std::vector<uint8_t> serializedData = packetptr->getBody();
    //should be able to access k_11 k_12 from pidcalibpacket
    //save that into the kp gains matrix
    std::string NVSName = "FRANCIS";
    NVSStore _NVS(NVSName, NVSStore::calibrationType::Servo);
    
    _NVS.saveBytes(serializedData);

    // READ FUN
};

void PID::check_gains() {
    std::string NVSName = "FRANCIS";
    NVSStore _NVS(NVSName, NVSStore::calibrationType::Servo);
    //load gains matrix and print it all to see if the calib command can change the values
    std::vector<uint8_t> calibSerialised = _NVS.loadBytes();
    if(calibSerialised.size() == 0)
    {
        // setNormalState(0); // default is nominally closed
        Serial.println("no data") ;// default NC
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
}

void PID::telemetry_impl(packetptr_t packetptr) {
    SimpleCommandPacket packet(*packetptr);

	PIDTelemetryPacket telemetry;

	telemetry.header.type = 108;
	telemetry.header.source = packet.header.destination;
	telemetry.header.source_service = m_serviceID;
	telemetry.header.destination = packet.header.source;
	telemetry.header.destination_service = packet.header.source_service;
	telemetry.header.uid = packet.header.uid; 
	telemetry.pitch_angle = m_output_values(0,0);
	telemetry.roll_angle = m_output_values(0,1);
	telemetry.prop_0 = m_output_values(0,2);
	telemetry.prop_1 = m_output_values(0,3);

	m_networkmanager.sendPacket(telemetry);

}