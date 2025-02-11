#pragma once

#include <librnp/rnp_packet.h>
#include <librnp/rnp_serializer.h>

#include <vector>


class PIDTelemetryPacket : public RnpPacket{
    private:
    //serializer framework
        static constexpr auto getSerializer()
        {
            auto ret = RnpSerializer(
                &PIDTelemetryPacket::pitch_angle,
                &PIDTelemetryPacket::roll_angle,
                &PIDTelemetryPacket::prop_0,
                &PIDTelemetryPacket::prop_1
            );

            return ret;
        }
        
    public:
        ~PIDTelemetryPacket();

        PIDTelemetryPacket();

        PIDTelemetryPacket(const RnpPacketSerialized& packet);

        void serialize(std::vector<uint8_t>& buf);// override;

        void deserializeBody(std::vector<uint8_t>& buf);

        float pitch_angle;
        float roll_angle;
        float prop_0;
        float prop_1;

        static constexpr size_t size(){
            return getSerializer().member_size();
        }

};