#pragma once

#include <librnp/rnp_packet.h>
#include <librnp/rnp_serializer.h>

#include <vector>


class PIDcalibrationpacket : public RnpPacket{
    private:
    //serializer framework
        static constexpr auto getSerializer()
        {
            auto ret = RnpSerializer(
                &PIDcalibrationpacket::PIDcontroller1,
                &PIDcalibrationpacket::PIDcontroller2,
                &PIDcalibrationpacket::PIDcontroller3,
                &PIDcalibrationpacket::K_11,
                &PIDcalibrationpacket::K_12,
                &PIDcalibrationpacket::K_13,
                &PIDcalibrationpacket::K_14,
                &PIDcalibrationpacket::K_15,
                &PIDcalibrationpacket::K_16,
                &PIDcalibrationpacket::K_21,
                &PIDcalibrationpacket::K_22,
                &PIDcalibrationpacket::K_23,
                &PIDcalibrationpacket::K_24,
                &PIDcalibrationpacket::K_25,
                &PIDcalibrationpacket::K_26,
                &PIDcalibrationpacket::K_31,
                &PIDcalibrationpacket::K_32,
                &PIDcalibrationpacket::K_33,
                &PIDcalibrationpacket::K_34,
                &PIDcalibrationpacket::K_35,
                &PIDcalibrationpacket::K_36,
                &PIDcalibrationpacket::K_41,
                &PIDcalibrationpacket::K_42,
                &PIDcalibrationpacket::K_43,
                &PIDcalibrationpacket::K_44,
                &PIDcalibrationpacket::K_45,
                &PIDcalibrationpacket::K_46
            );

            return ret;
        }
        
    public:
        ~PIDcalibrationpacket();

        PIDcalibrationpacket();
        /**
         * @brief Deserialize Packet
         * 
         * @param data 
         */
        PIDcalibrationpacket(const RnpPacketSerialized& packet);

        /**
         * @brief Serialize Packet
         */
        void serialize(std::vector<uint8_t>& buf);// override;

        void deserializeBody(std::vector<uint8_t>& buf);

        uint8_t PIDcontroller1;

        /**
         * @brief controller 1 matrix
         * 
         */
         uint8_t PIDcontroller2;

        /**
         * @brief controller 2 matrix
         * 
         */
        
        uint8_t PIDcontroller3;

        /**
         * @brief controller 2 matrix
         * 
         */
        uint32_t K_11;
        /**
         * @brief servo1 K_x
         * 
         */
        uint32_t K_12;
        /**
         * @brief servo1 K_y
         * 
         */
        uint32_t K_13;
        /**
         * @brief servo1 K_z
         * 
         */
        uint32_t K_14;
        /**
         * @brief servo1 K_roll
         * 
         */
        uint32_t K_15;
        /**
         * @brief servo1 K_pitch
         * 
         */
        uint32_t K_16;
        /**
         * @brief servo1 K_yaw
         * 
         */
        uint32_t K_21;
        /**
         * @brief servo2 K_x
         * 
         */
        uint32_t K_22;
        /**
         * @brief servo2 K_y
         * 
         */
        uint32_t K_23;
        /**
         * @brief servo2 K_z
         * 
         */
        uint32_t K_24;
        /**
         * @brief servo2 K_roll
         * 
         */
        uint32_t K_25;
        /**
         * @brief servo2 K_pitch
         * 
         */
        uint32_t K_26;
        /**
         * @brief servo2 K_yaw
         * 
         */
        uint32_t K_31;
        /**
         * @brief propeller1 K_x
         * 
         */
        uint32_t K_32;
        /**
         * @brief propeller1 K_y
         * 
         */
        uint32_t K_33;
        /**
         * @brief propeller1 K_z
         * 
         */
        uint32_t K_34;
        /**
         * @brief propeller1 K_roll
         * 
         */
        uint32_t K_35;
        /**
         * @brief propeller1 K_pitch
         * 
         */
        uint32_t K_36;
        /**
         * @brief propeller1 K_yaw
         * 
         */
        uint32_t K_41;
        /**
         * @brief propeller2 K_x
         * 
         */
        uint32_t K_42;
        /**
         * @brief propeller2 K_y
         * 
         */
        uint32_t K_43;
        /**
         * @brief propeller2 K_z
         * 
         */
        uint32_t K_44;
        /**
         * @brief propeller2 K_roll
         * 
         */
        uint32_t K_45;
        /**
         * @brief propeller2 K_pitch
         * 
         */
        uint32_t K_46;
        /**
         * @brief propeller2 K_yaw
         * 
         */
        static constexpr size_t size(){
            return getSerializer().member_size();
        }

};