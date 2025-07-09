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
                &ControllerTelemetryPacket::x_input,
                &ControllerTelemetryPacket::y_input,
                &ControllerTelemetryPacket::z_input,
                &ControllerTelemetryPacket::u_input,
                &ControllerTelemetryPacket::v_input,
                &ControllerTelemetryPacket::w_input,
                &ControllerTelemetryPacket::roll_input,
                &ControllerTelemetryPacket::pitch_input,
                &ControllerTelemetryPacket::yaw_input,
                &ControllerTelemetryPacket::roll_rate_input,
                &ControllerTelemetryPacket::pitch_rate_input,
                &ControllerTelemetryPacket::yaw_rate_input,
                &ControllerTelemetryPacket::pitch_output,
                &ControllerTelemetryPacket::roll_output,
                &ControllerTelemetryPacket::prop_0,
                &ControllerTelemetryPacket::prop_1
            );

            return ret;
        }
        
    public:
        ~ControllerTelemetryPacket();

        ControllerTelemetryPacket();

        ControllerTelemetryPacket(const RnpPacketSerialized& packet);

        void serialize(std::vector<uint8_t>& buf);// override;

        void deserializeBody(std::vector<uint8_t>& buf);

        float x_input;
        float y_input;
        float z_input;
        float u_input;
        float v_input;
        float w_input;
        float roll_input;
        float pitch_input;
        float yaw_input;
        float roll_rate_input;
        float pitch_rate_input;
        float yaw_rate_input;
        float roll_output;
        float pitch_output;
        float prop_0;
        float prop_1;

        static constexpr size_t size(){
            return getSerializer().member_size();
        }

};