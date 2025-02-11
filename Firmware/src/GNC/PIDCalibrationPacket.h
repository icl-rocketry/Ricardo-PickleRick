#pragma once

#include <librnp/rnp_packet.h>
#include <librnp/rnp_serializer.h>

#include <vector>


class PIDCalibrationPacket : public RnpPacket{
    private:
    //serializer framework
        static constexpr auto getSerializer()
        {
            auto ret = RnpSerializer(
                &PIDCalibrationPacket::K_11,
                &PIDCalibrationPacket::K_12,
                &PIDCalibrationPacket::K_13,
                &PIDCalibrationPacket::K_14,
                &PIDCalibrationPacket::K_15,
                &PIDCalibrationPacket::K_16
                // &PIDCalibrationPacket::K_21,
                // &PIDCalibrationPacket::K_22,
                // &PIDCalibrationPacket::K_23,
                // &PIDCalibrationPacket::K_24,
                // &PIDCalibrationPacket::K_25,
                // &PIDCalibrationPacket::K_26,
                // &PIDCalibrationPacket::K_31,
                // &PIDCalibrationPacket::K_32,
                // &PIDCalibrationPacket::K_33,
                // &PIDCalibrationPacket::K_34,
                // &PIDCalibrationPacket::K_35,
                // &PIDCalibrationPacket::K_36,
                // &PIDCalibrationPacket::K_41,
                // &PIDCalibrationPacket::K_42,
                // &PIDCalibrationPacket::K_43,
                // &PIDCalibrationPacket::K_44,
                // &PIDCalibrationPacket::K_45,
                // &PIDCalibrationPacket::K_46
            );

            return ret;
        }
        
    public:
        ~PIDCalibrationPacket();

        PIDCalibrationPacket();
        /**
         * @brief Deserialize Packet
         * 
         * @param data 
         */
        PIDCalibrationPacket(const RnpPacketSerialized& packet);

        /**
         * @brief Serialize Packet
         */
        void serialize(std::vector<uint8_t>& buf);// override;

        void deserializeBody(std::vector<uint8_t>& buf);

        float K_11;
   
        float K_12;

        float K_13;
  
        float K_14;

        float K_15;

        float K_16;

 
        static constexpr size_t size(){
            return getSerializer().member_size();
        }

};