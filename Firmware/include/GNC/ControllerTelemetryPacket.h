#pragma once

#include <librnp/rnp_packet.h>
#include <librnp/rnp_serializer.h>

#include <string>
#include <vector>

class ControllerTelemetryPacket : public RnpPacket{
    private:
    //serializer framework
        static constexpr auto getSerializer()
        {
            auto ret = RnpSerializer(
                &ControllerTelemetryPacket::q0,
                &ControllerTelemetryPacket::q1,
                &ControllerTelemetryPacket::q2,
                &ControllerTelemetryPacket::q3,
                &ControllerTelemetryPacket::total_error,
                &ControllerTelemetryPacket::pitch_error,
                &ControllerTelemetryPacket::yaw_error,
                &ControllerTelemetryPacket::roll_rate_input,
                &ControllerTelemetryPacket::pitch_rate_input,
                &ControllerTelemetryPacket::yaw_rate_input,
                &ControllerTelemetryPacket::pitch_output,
                &ControllerTelemetryPacket::yaw_output,
                &ControllerTelemetryPacket::thrust_top,
                &ControllerTelemetryPacket::thrust_bottom,
                &ControllerTelemetryPacket::fx_body,
                &ControllerTelemetryPacket::fy_body,
                &ControllerTelemetryPacket::fz_body,
                &ControllerTelemetryPacket::m_cmd_y,
                &ControllerTelemetryPacket::m_cmd_z,
                &ControllerTelemetryPacket::m_roll_mix,
                &ControllerTelemetryPacket::m_batt,
                &ControllerTelemetryPacket::m_pos_err_dbg_x,
                &ControllerTelemetryPacket::m_pos_err_dbg_y,
                &ControllerTelemetryPacket::m_pos_err_dbg_z,
                &ControllerTelemetryPacket::m_vel_err_dbg_x,
                &ControllerTelemetryPacket::m_vel_err_dbg_y,
                &ControllerTelemetryPacket::m_vel_err_dbg_z,
                &ControllerTelemetryPacket::m_pos_des_x,
                &ControllerTelemetryPacket::m_pos_des_y,
                &ControllerTelemetryPacket::m_pos_des_z,
                &ControllerTelemetryPacket::m_vel_des_x,
                &ControllerTelemetryPacket::m_vel_des_y,
                &ControllerTelemetryPacket::m_vel_des_z,
                &ControllerTelemetryPacket::m_acc_des_x,
                &ControllerTelemetryPacket::m_acc_des_y,
                &ControllerTelemetryPacket::m_acc_des_z,
                &ControllerTelemetryPacket::m_thrust_world_des_x,
                &ControllerTelemetryPacket::m_thrust_world_des_y,
                &ControllerTelemetryPacket::m_thrust_world_des_z,
                &ControllerTelemetryPacket::system_time
            );

            return ret;
        }
        
    public:
        ~ControllerTelemetryPacket();

        ControllerTelemetryPacket();

        ControllerTelemetryPacket(const RnpPacketSerialized& packet);

        void serialize(std::vector<uint8_t>& buf);// override;

        void deserializeBody(std::vector<uint8_t>& buf);

        std::string stringify() const;

        // float x_input;
        // float y_input;
        // float z_input;
        // float u_input;
        // float v_input;
        // float w_input;
        // float roll_input;
        // float pitch_input;
        // float yaw_input;
        float q0;
        float q1;
        float q2;
        float q3;
        float total_error;   // total thrust-axis error (deg); roll about thrust axis is not observed
        float pitch_error;  // signed thrust-vector pitch channel error (deg)
        float yaw_error;    // signed thrust-vector yaw channel error (deg)
        float roll_rate_input;
        float pitch_rate_input;
        float yaw_rate_input;
        float pitch_output;
        float yaw_output;
        float thrust_top;
        float thrust_bottom;
        float fx_body;
        float fy_body;
        float fz_body;
        float m_cmd_y;
        float m_cmd_z;
        float m_roll_mix;
        float m_batt;
        float m_pos_err_dbg_x;
        float m_pos_err_dbg_y;
        float m_pos_err_dbg_z;
        float m_vel_err_dbg_x;
        float m_vel_err_dbg_y;
        float m_vel_err_dbg_z;
        float m_pos_des_x;
        float m_pos_des_y;
        float m_pos_des_z;
        float m_vel_des_x;
        float m_vel_des_y;
        float m_vel_des_z;
        float m_acc_des_x;
        float m_acc_des_y;
        float m_acc_des_z;
        float m_thrust_world_des_x;
        float m_thrust_world_des_y;
        float m_thrust_world_des_z;
        uint32_t system_time;
        static constexpr size_t size(){
            return getSerializer().member_size();
        }

};
