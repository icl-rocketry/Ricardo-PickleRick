#pragma once

#include <librnp/rnp_packet.h>
#include <librnp/rnp_serializer.h>

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
                &ControllerTelemetryPacket::roll_rate_input,
                &ControllerTelemetryPacket::pitch_rate_input,
                &ControllerTelemetryPacket::yaw_rate_input,
                &ControllerTelemetryPacket::pitch_output,
                &ControllerTelemetryPacket::roll_output,
                &ControllerTelemetryPacket::thrust
            );

            return ret;
        }
        
    public:
        ~ControllerTelemetryPacket();

        ControllerTelemetryPacket();

        ControllerTelemetryPacket(const RnpPacketSerialized& packet);

        void serialize(std::vector<uint8_t>& buf);// override;

        void deserializeBody(std::vector<uint8_t>& buf);

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
        float roll_rate_input;
        float pitch_rate_input;
        float yaw_rate_input;
        float roll_output;
        float pitch_output;
        float thrust;

        static constexpr size_t size(){
            return getSerializer().member_size();
        }

};