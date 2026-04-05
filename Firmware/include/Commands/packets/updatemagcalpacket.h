#pragma once

#include <Eigen/Core>

#include <librnp/rnp_packet.h>
#include <librnp/rnp_serializer.h>
#include <librnp/default_packets/simplecommandpacket.h>

class UpdateMagCalPacket: public RnpPacket{
    private:
        static constexpr auto getSerializer()
        {
            auto ret = RnpSerializer(
                &UpdateMagCalPacket::command,
                &UpdateMagCalPacket::A11,
                &UpdateMagCalPacket::A12,
                &UpdateMagCalPacket::A13,
                &UpdateMagCalPacket::A21,
                &UpdateMagCalPacket::A22,
                &UpdateMagCalPacket::A23,
                &UpdateMagCalPacket::A31,
                &UpdateMagCalPacket::A32,
                &UpdateMagCalPacket::A33,
                &UpdateMagCalPacket::b1,
                &UpdateMagCalPacket::b2,
                &UpdateMagCalPacket::b3
            );
            return ret;
        }
    public:
        ~UpdateMagCalPacket();
        UpdateMagCalPacket(uint8_t command);

        /**
         * @brief Deserialize Command Packet from serialized data
         * 
         * @param packet 
         */
        UpdateMagCalPacket(const RnpPacketSerialized& packet);

        /**
         * @brief Serialize into provided buffer
         * 
         * @param buf 
         */
        void serialize(std::vector<uint8_t>& buf) override;

        /**
         * @brief get A matrix as a matrix object
         * 
         * @return Eigen::Matrix3f 
         */
        Eigen::Matrix3f getA();
        /**
         * @brief get B vecotr as a vector object
         * 
         * @return Eigen::Vector3f 
         */
        Eigen::Vector3f getB();

        //data members
        command_t command;

        float A11;
        float A12;
        float A13;
        float A21;
        float A22;
        float A23;
        float A31;
        float A32;
        float A33;
        float b1;
        float b2;
        float b3;


        
        static constexpr size_t size(){
            return getSerializer().member_size();
        }
};


