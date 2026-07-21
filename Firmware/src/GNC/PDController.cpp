#include "GNC/PDController.h"
#include "Config/controller_config.h"
#include "Config/debug_config.h"
#include <algorithm>
#include <cmath>

namespace {
constexpr uint32_t CONTROLLER_FRAME_DEBUG_PRINT_PERIOD_MS = 250;
}

void PDController::setup()
{
    // Desired thrust direction in WORLD frame
    // Change this to match your world convention.
    m_thrust_dir_world_des << 0.0f, 0.0f, -1.0f;

    // m_rEng is COM -> thrust centre in body frame (m). Tune ry/rz for attitude-only lateral drift:
    // with rx < 0, negative body-y drift -> make ry more positive; positive body-y drift -> make ry more negative.
    // negative body-z drift -> make rz more positive; positive body-z drift -> make rz more negative.
    m_rEng << -0.235f, 0.004f, 0.001f;
    m_mass = 1.36f;

    m_K_p << 3.0f, 3.0f, 2.8f; // attitude body control gains (roll, pitch, yaw)
    m_K_d << 7.0f, 1.0f, 0.9f;

    // m_K_p_pos << 0.5f, 0.5f, 0.45f;   //best so far
    // m_K_d_pos << 2.6f, 2.6f, 3.3f; 
    // m_K_i_pos << 0.075f, 0.075f, 0.07f;

    // m_K_p_pos << 0.45f, 0.45f, 1.3f;   // NED position control gains
    // m_K_d_pos << 2.6f, 2.6f, 3.7f; 
    // m_K_i_pos << 0.02f, 0.02f, 0.01f;

    m_K_p_pos << 0.3f, 0.3f, 2.5f;   // NED position control gains
    m_K_d_pos << 2.0f, 2.0f, 3.5f; 
    m_K_i_pos << 0.0f, 0.0f, 0.0f;


    m_pos_int.setZero();
    m_pos_des << 0.0f, 0.0f, 0.0f; 
    m_vel_des.setZero();
    m_acc_des.setZero();
    m_position_dbg.setZero();
    m_velocity_dbg.setZero();
    m_thrust_dir_world_raw_dbg.setZero();
    m_thrust_dir_body_des_dbg.setZero();
    m_body_x_world_dbg.setZero();
    m_body_y_world_dbg.setZero();
    m_body_z_world_dbg.setZero();
    m_pos_err_dbg.setZero();
    m_vel_err_dbg.setZero();
    m_roll_zero_body_x_world.setZero();
    m_roll_zero_body_y_world.setZero();
    m_roll_zero_valid = false;
    m_euler_error.setZero();
    m_thrust_vector_error_deg.setZero();
    m_max_vel       = 1.0f;                  // m/s — conservative
    m_max_tilt_rad  = MAX_POSITION_TILT_RAD;
    m_last_update_us = 0;

    m_position_control_enabled = true;      // arm explicitly
    m_Fx_cmd_outer  = NOMINAL_FX_N;
}

void PDController::setPositionTarget(const Eigen::Vector3f& position,
                                     const Eigen::Vector3f& velocity,
                                     const Eigen::Vector3f& acceleration)
{
    m_pos_des = position;
    m_vel_des = velocity;
    m_acc_des = acceleration;
}

void PDController::setPositionControlEnabled(bool enabled)
{
    if (!enabled) {
        m_pos_int.setZero();
    }

    m_position_control_enabled = enabled;
}

void PDController::update(Eigen::Quaterniond q, 
                          Eigen::Vector3f angular_rates, 
                          Eigen::Vector3f position, //NED
                          Eigen::Vector3f velocity, //NED
                          float batt_V, bool batt_fresh)
{
    m_batt_V = batt_V;
    m_batt_fresh = batt_fresh;
    q.normalize();
    updatePositionControl(position, velocity);
    updateThrustDirectionErrors(q);
    updateDesiredForce(q);
    updateMcmd(angular_rates);
    updateOutputValues(); 
}

void PDController::reset()
{
    m_dir_error_body << 0.0f, 0.0f, 0.0f;
    m_M_cmd          << 0.0f, 0.0f, 0.0f;
    m_output_values  << 0.0f, 0.0f, 0.0f, 0.0f;
    m_Fx_cmd         = 0.0f;
    m_voltage_scale  = 1.0f;
    m_euler_error.setZero();
    m_pos_int.setZero();
    m_thrust_vector_error_deg.setZero();
    m_position_dbg.setZero();
    m_velocity_dbg.setZero();
    m_thrust_dir_world_raw_dbg.setZero();
    m_thrust_dir_body_des_dbg.setZero();
    m_body_x_world_dbg.setZero();
    m_body_y_world_dbg.setZero();
    m_body_z_world_dbg.setZero();
    m_pos_err_dbg.setZero();
    m_vel_err_dbg.setZero();
    m_roll_zero_body_x_world.setZero();
    m_roll_zero_body_y_world.setZero();
    m_roll_zero_valid = false;
}

void PDController::updatePositionControl(const Eigen::Vector3f& position,
    const Eigen::Vector3f& velocity)
{
    m_position_dbg = position;
    m_velocity_dbg = velocity;

    // ── dt ───────────────────────────────────────────────────────────────
    const uint32_t now = micros();

    if (m_last_update_us == 0) {

        m_last_update_us = now;
        m_dt = 0.0f;
        return;

    }

    m_dt = (now - m_last_update_us) * 1e-6f;
    m_last_update_us = now;

    if (m_dt <= 0.0f || m_dt > 0.1f) {return;}

    if (!m_position_control_enabled) {

        m_pos_int.setZero();
        m_thrust_dir_world_des << 0.0f, 0.0f, -1.0f;   // -D is up in World Frame 
        m_thrust_dir_world_raw_dbg = m_thrust_dir_world_des;
        m_Fx_cmd_outer = NOMINAL_FX_N;
        return;

    }

    // ── Simple PID: position error → acceleration command ────────────────
    Eigen::Vector3f pos_err = m_pos_des - position;

    Eigen::Vector3f vel_err = m_vel_des - velocity;

    // Integrator
    m_pos_int += pos_err * m_dt;

    //anti windup for integrator
    const float I_MAX = 20.0f;
    m_pos_int = m_pos_int.cwiseMax(-I_MAX).cwiseMin(I_MAX);

    Eigen::Vector3f acceleration_feedforward = m_acc_des;
    if (!ControllerConfig::VerticalAccelerationFeedforwardEnabled) {
        acceleration_feedforward(2) = 0.0f;
    }

    Eigen::Vector3f a_des =
        acceleration_feedforward
        + m_K_p_pos.cwiseProduct(pos_err)
        + m_K_i_pos.cwiseProduct(m_pos_int)
        + m_K_d_pos.cwiseProduct(vel_err);

    // Gravity compensation: -D is up
    a_des += Eigen::Vector3f(0.0f, 0.0f, -GRAVITY);

    // ── Convert acceleration command to thrust vector ────────────────────
    Eigen::Vector3f F_des_world = limitPositionTiltRequest(m_mass * a_des);
    const Eigen::Vector3f F_des_world_raw = F_des_world;
    float F_mag = F_des_world.norm();
    const float F_mag_raw = F_des_world_raw.norm();

    if (F_mag < 1e-3f) {
        m_thrust_dir_world_des << 0.0f, 0.0f, -1.0f;
        m_thrust_dir_world_raw_dbg = m_thrust_dir_world_des;
        m_Fx_cmd_outer = 0.0f;
    } else {
        if (F_mag_raw < 1e-3f) {
            m_thrust_dir_world_raw_dbg << 0.0f, 0.0f, -1.0f;
        } else {
            m_thrust_dir_world_raw_dbg = F_des_world_raw / F_mag_raw;
        }

        Eigen::Vector3f dir = F_des_world / F_mag;

        m_thrust_dir_world_des = dir;
        m_Fx_cmd_outer = std::clamp(F_mag, 0.0f, MAX_THRUST_N);
    }
    
    // Telemetry
    m_position_dbg = position;
    m_velocity_dbg = velocity;
    m_pos_err_dbg = pos_err;
    m_vel_err_dbg = vel_err;

}

Eigen::Vector3f PDController::limitPositionTiltRequest(const Eigen::Vector3f& force_world) const
{
    Eigen::Vector3f limited_force = force_world;

    const float upward_force = std::max(-limited_force(2), 0.0f);
    limited_force(2) = -upward_force;

    const float max_horizontal_force = upward_force * std::tan(m_max_tilt_rad);
    const float horizontal_force = limited_force.head<2>().norm();

    if (horizontal_force > max_horizontal_force) {
        if (horizontal_force > 1e-6f) {
            limited_force.head<2>() *= max_horizontal_force / horizontal_force;
        } else {
            limited_force.head<2>().setZero();
        }
    }

    return limited_force;
}

void PDController::updateThrustDirectionErrors(const Eigen::Quaterniond& q)
{
    captureRollZeroReference(q);

    // Body thrust axis = +x_body
    const Eigen::Vector3d thrust_axis_body(1.0, 0.0, 0.0);

    const Eigen::Vector3d body_y_world = q * Eigen::Vector3d::UnitY();
    const Eigen::Vector3d body_z_world = q * Eigen::Vector3d::UnitZ();

    // Current thrust direction in WORLD frame
    Eigen::Vector3d thrust_dir_world = q * thrust_axis_body;
    thrust_dir_world.normalize();
    m_body_x_world_dbg = thrust_dir_world.cast<float>();
    m_body_y_world_dbg = body_y_world.cast<float>();
    m_body_z_world_dbg = body_z_world.cast<float>();

    // Desired thrust direction in WORLD frame
    Eigen::Vector3d thrust_dir_world_des = m_thrust_dir_world_des.cast<double>();
    if (thrust_dir_world_des.norm() < 1e-6) {
        thrust_dir_world_des << 0.0, 0.0, -1.0;
    }
    thrust_dir_world_des.normalize();

    // Error axis in WORLD frame
    // This is zero when the vectors align, and ignores roll about thrust axis
    Eigen::Vector3d e_world = thrust_dir_world.cross(thrust_dir_world_des);

    // Convert error into BODY frame so it matches body rates and actuator axes
    Eigen::Vector3d e_body = q.conjugate() * e_world;

    m_dir_error_body = e_body.cast<float>();

    const Eigen::Vector3d desired_body = q.conjugate() * thrust_dir_world_des;
    m_thrust_dir_body_des_dbg = desired_body.cast<float>();

    //send to telemetry
    const double total_error_rad = std::asin(std::clamp(e_world.norm(), 0.0, 1.0));
    const double pitch_error_rad = std::atan2(desired_body.z(), desired_body.x());
    const double yaw_error_rad   = std::atan2(desired_body.y(), desired_body.x());

    m_thrust_vector_error_deg << static_cast<float>(total_error_rad * RAD_TO_DEG),
                                 static_cast<float>(pitch_error_rad * RAD_TO_DEG),
                                 static_cast<float>(yaw_error_rad * RAD_TO_DEG);

    Eigen::Vector3d roll_zero_x_world = m_roll_zero_body_x_world.cast<double>();
    Eigen::Vector3d roll_zero_y_world = m_roll_zero_body_y_world.cast<double>();
    if (roll_zero_x_world.norm() < 1e-6 || roll_zero_y_world.norm() < 1e-6) {
        roll_zero_x_world = thrust_dir_world;
        roll_zero_y_world = body_y_world;
    }
    roll_zero_x_world.normalize();
    roll_zero_y_world.normalize();

    const Eigen::Quaterniond zero_to_current_thrust =
        Eigen::Quaterniond::FromTwoVectors(roll_zero_x_world, thrust_dir_world);

    Eigen::Vector3d roll_reference_y_world = zero_to_current_thrust * roll_zero_y_world;
    roll_reference_y_world -= thrust_dir_world * roll_reference_y_world.dot(thrust_dir_world);

    Eigen::Vector3d current_y_world = body_y_world;
    current_y_world -= thrust_dir_world * current_y_world.dot(thrust_dir_world);

    if (roll_reference_y_world.norm() > 1e-6 && current_y_world.norm() > 1e-6) {
        roll_reference_y_world.normalize();
        current_y_world.normalize();

        const double roll_error_rad = std::atan2(
            thrust_dir_world.dot(current_y_world.cross(roll_reference_y_world)),
            current_y_world.dot(roll_reference_y_world));

        m_dir_error_body(0) = static_cast<float>(roll_error_rad);
    } else {
        m_dir_error_body(0) = 0.0f;
    }
}

void PDController::captureRollZeroReference(const Eigen::Quaterniond& q)
{
    if (m_roll_zero_valid) {
        return;
    }

    m_roll_zero_body_x_world = (q * Eigen::Vector3d::UnitX()).normalized().cast<float>();
    m_roll_zero_body_y_world = (q * Eigen::Vector3d::UnitY()).normalized().cast<float>();
    m_roll_zero_valid = true;
}

void PDController::updateMcmd(const Eigen::Vector3f& angular_rates)
{
    Eigen::Vector3f rates_error = -angular_rates; // desired rates are zero
    m_euler_error = m_dir_error_body;//send the errors to telemetry for debugging
    m_M_cmd =
        m_K_p.cwiseProduct(m_dir_error_body)
        + m_K_d.cwiseProduct(rates_error);    
   // m_M_cmd = 0.0f * m_K_p.cwiseProduct(m_dir_error_body) + 0.0f * m_K_d.cwiseProduct(rates_error); //disable attitude control for now, just use position control
}
void PDController::updateDesiredForce(const Eigen::Quaterniond& q)
{
    const float Fx_target = m_position_control_enabled ? m_Fx_cmd_outer : NOMINAL_FX_N;

    if (!m_position_control_enabled) {
        m_Fx_cmd = std::clamp(Fx_target, 0.0f, MAX_THRUST_N);
        return;
    }

    Eigen::Vector3d thrust_dir_world_des = m_thrust_dir_world_des.cast<double>();
    if (thrust_dir_world_des.norm() < 1e-6) {
        thrust_dir_world_des << 0.0, 0.0, -1.0;
    }
    thrust_dir_world_des.normalize();

    const Eigen::Vector3d body_x_world = (q * Eigen::Vector3d::UnitX()).normalized();
    const Eigen::Vector3d upright_thrust_world(0.0, 0.0, -1.0);
    const float cos_tilt = std::clamp(
        static_cast<float>(body_x_world.dot(upright_thrust_world)),
        TILT_COMPENSATION_MIN_COS,
        1.0f);
    const float upward_force_target =
        std::max(-Fx_target * static_cast<float>(thrust_dir_world_des.z()), 0.0f);

    m_Fx_cmd = std::clamp(upward_force_target / cos_tilt, 0.0f, MAX_THRUST_N);
}
void PDController::updateOutputValues()
{
    //take in the centre of mass offsets for the engine 
    const float rx = m_rEng(0);
    const float ry = m_rEng(1);
    const float rz = m_rEng(2);

    //compute desired body forces
    
    Eigen::Vector3f F_body;
    F_body(0) = m_Fx_cmd;
    F_body(1) = (m_M_cmd(2) + ry * m_Fx_cmd) / rx;
    F_body(2) = (rz * m_Fx_cmd - m_M_cmd(1)) / rx;
   
    // Convert desired body forces into servo angles and thrust commands.
    float pitch_servo =  std::atan2(F_body(2), F_body(0)) * RAD_TO_DEG; //i.e. controlling the pitch of the vehicle 
    float yaw_servo   =  std::atan2(F_body(1), F_body(0)) * RAD_TO_DEG; //i.e controlling the yaw of the vehicle 
    float base_thrust = sqrtf(F_body(0)*F_body(0) + F_body(1)*F_body(1) + F_body(2)*F_body(2)) * 100.0f / MAX_THRUST_N;
    
    //--------VOLTAGE SCALING-----------------
    float voltage_scale = 1.0f;
    if (m_batt_fresh && m_batt_V > MIN_VALID_BATT_V)
    {
        const float raw_voltage_scale = NOMINAL_BATT_V / m_batt_V;
        voltage_scale = powf(raw_voltage_scale, VOLTAGE_SCALE_EXPONENT);
        voltage_scale = std::clamp(voltage_scale, MIN_VOLTAGE_SCALE, MAX_VOLTAGE_SCALE);
    }
    m_voltage_scale = voltage_scale;

    base_thrust *= voltage_scale; //scale thrust based on voltage read 

    pitch_servo = std::clamp(pitch_servo, -MAX_GIMBAL_DEG, MAX_GIMBAL_DEG);
    yaw_servo   = std::clamp(yaw_servo,   -MAX_GIMBAL_DEG, MAX_GIMBAL_DEG);
    base_thrust = std::clamp(base_thrust, 0.0f, 100.0f);

    //_--------ROLL CONTROL-----------------
    // Roll angle control and rate damping via differential prop throttle.
    // Positive roll_mix: top CW prop up, bottom CCW prop down.
    float roll_mix = m_M_cmd(0);
    roll_mix *= voltage_scale; //voltage scaling for roll mix 
    roll_mix += ROLL_MIX_OFFSET; //constant trim to counter negative roll bias
    roll_mix = std::clamp(roll_mix, -MAX_ROLL_MIX, MAX_ROLL_MIX);
   
    float thrust_top = std::clamp(base_thrust + roll_mix, 0.0f, 100.0f);
    float thrust_bottom = std::clamp(base_thrust - roll_mix, 0.0f, 100.0f);

    //--------Thrust Linerisation----------------- 
    thrust_top = 100.0f * powf(thrust_top/ 100.0f, THRUST_EXPONENT);
    thrust_bottom = 100.0f * powf(thrust_bottom / 100.0f, THRUST_EXPONENT);


    //Sending values to telemetry 
    m_roll_mix = roll_mix; //send the roll mix to telemetry for debugging
    m_f_body = F_body; //send the body forces to telemetry for debugging

    m_output_values << pitch_servo,
                       yaw_servo,
                       thrust_top,
                       thrust_bottom;

    if constexpr (DebugConfig::ControllerFramePrintEnabled)
    {
        static uint32_t last_controller_frame_debug_print_ms = 0;
        const uint32_t now_ms = millis();
        if (now_ms - last_controller_frame_debug_print_ms >= CONTROLLER_FRAME_DEBUG_PRINT_PERIOD_MS) {
            last_controller_frame_debug_print_ms = now_ms;
            Serial.printf(
                "CTRL_FRAME "
                "pos_ned=(%.3f,%.3f,%.3f) target_ned=(%.3f,%.3f,%.3f) err_ned=(%.3f,%.3f,%.3f) "
                "thrust_world_raw=(%.3f,%.3f,%.3f) thrust_world_cmd=(%.3f,%.3f,%.3f) "
                "body_x_world=(%.3f,%.3f,%.3f) body_y_world=(%.3f,%.3f,%.3f) body_z_world=(%.3f,%.3f,%.3f) "
                "thrust_body_des=(%.3f,%.3f,%.3f) dir_err_body=(%.3f,%.3f,%.3f) "
                "M_cmd=(%.3f,%.3f,%.3f) F_body=(%.3f,%.3f,%.3f) "
                "pitch_out=%.3f yaw_out=%.3f servo_top=%.3f servo_bottom=%.3f\n",
                m_position_dbg(0), m_position_dbg(1), m_position_dbg(2),
                m_pos_des(0), m_pos_des(1), m_pos_des(2),
                m_pos_err_dbg(0), m_pos_err_dbg(1), m_pos_err_dbg(2),
                m_thrust_dir_world_raw_dbg(0), m_thrust_dir_world_raw_dbg(1), m_thrust_dir_world_raw_dbg(2),
                m_thrust_dir_world_des(0), m_thrust_dir_world_des(1), m_thrust_dir_world_des(2),
                m_body_x_world_dbg(0), m_body_x_world_dbg(1), m_body_x_world_dbg(2),
                m_body_y_world_dbg(0), m_body_y_world_dbg(1), m_body_y_world_dbg(2),
                m_body_z_world_dbg(0), m_body_z_world_dbg(1), m_body_z_world_dbg(2),
                m_thrust_dir_body_des_dbg(0), m_thrust_dir_body_des_dbg(1), m_thrust_dir_body_des_dbg(2),
                m_dir_error_body(0), m_dir_error_body(1), m_dir_error_body(2),
                m_M_cmd(0), m_M_cmd(1), m_M_cmd(2),
                F_body(0), F_body(1), F_body(2),
                pitch_servo, yaw_servo,
                pitch_servo, -yaw_servo);
        }
    }
}
