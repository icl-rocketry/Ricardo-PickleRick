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