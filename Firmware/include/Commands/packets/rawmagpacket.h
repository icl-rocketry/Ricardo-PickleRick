#pragma once

#include <librnp/rnp_packet.h>
#include <librnp/rnp_serializer.h>

class RawMagPacket: public RnpPacket{
    private:
    //serializer framework
        static constexpr auto getSerializer()
        {
            auto ret = RnpSerializer(
                &RawMagPacket::mx,
                &RawMagPacket::my,
                &RawMagPacket::mz
            );
            return ret;
        }

    public:
        ~RawMagPacket();

        RawMagPacket();
        /**
         * @brief Deserialize Mag Packet
         * 
         * @param data 
         */
        RawMagPacket(const RnpPacketSerialized& packet);

        /**
         * @brief Serialize into provided buffer
         * 
         * @param buf 
         */
        void serialize(std::vector<uint8_t>& buf) override;

        float mx;
        float my;
        float mz;


        
        static constexpr size_t size(){
            return getSerializer().member_size();
        }
};


