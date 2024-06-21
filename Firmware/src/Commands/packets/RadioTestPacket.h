#pragma once

#include <librnp/rnp_packet.h>
#include <librnp/rnp_serializer.h>

#include <vector>

class RadioTestPacket : public RnpPacket{
    private:
    //serializer framework
        static constexpr auto getSerializer()
        {
            auto ret = RnpSerializer(
                &RadioTestPacket::system_time,
                &RadioTestPacket::system_status,
                &RadioTestPacket::rssi,
                &RadioTestPacket::packet_rssi,
                &RadioTestPacket::snr,
                &RadioTestPacket::packet_snr
            );
            return ret;
        }
        
    public:
        ~RadioTestPacket();

        RadioTestPacket();
        /**
         * @brief Deserialize Telemetry Packet
         * 
         * @param data 
         */
        RadioTestPacket(const RnpPacketSerialized& packet);

        /**
         * @brief Serialize Telemetry Packet
         * 
         * @param buf 
         */
        void serialize(std::vector<uint8_t>& buf) override;

        
        uint64_t system_time;
        uint32_t system_status;
        //radio details
        int16_t rssi, packet_rssi;
        float snr, packet_snr;


        static constexpr size_t size(){
            return getSerializer().member_size();
        }

};


