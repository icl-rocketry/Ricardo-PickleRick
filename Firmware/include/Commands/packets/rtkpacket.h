#pragma once

#include <librnp/rnp_packet.h>
#include <librnp/rnp_serializer.h>

#include <cstdint>
#include <vector>


class RTKPacket : public RnpPacket{
    private:
    //serializer framework
        static constexpr auto getSerializer()
        {
            auto ret = RnpSerializer(
                &RTKPacket::x_input,
                &RTKPacket::y_input,
                &RTKPacket::z_input,
                &RTKPacket::u_input,
                &RTKPacket::v_input,
                &RTKPacket::w_input,
                &RTKPacket::fix_quality,
                &RTKPacket::wifi_connected,
                &RTKPacket::gnss_time_of_day_ms
            );

            return ret;
        }
        
    public:
        ~RTKPacket();

        RTKPacket();

        RTKPacket(const RnpPacketSerialized& packet);

        void serialize(std::vector<uint8_t>& buf);// override;

        void deserializeBody(std::vector<uint8_t>& buf);

        float x_input;
        float y_input;
        float z_input;
        float u_input;
        float v_input;
        float w_input;
        uint8_t fix_quality;
        uint8_t wifi_connected;
        uint32_t gnss_time_of_day_ms;

        static constexpr size_t size(){
            return getSerializer().member_size();
        }

};
