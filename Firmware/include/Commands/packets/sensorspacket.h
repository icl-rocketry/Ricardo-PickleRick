#pragma once

#include <librnp/rnp_packet.h>
#include <librnp/rnp_serializer.h>

#include <vector>

class SensorsPacket : public RnpPacket{
    private:
    //serializer framework
        static constexpr auto getSerializer()
        {
            auto ret = RnpSerializer(
                &SensorsPacket::ax,
                &SensorsPacket::ay,
                &SensorsPacket::az,
                &SensorsPacket::gx,
                &SensorsPacket::gy,
                &SensorsPacket::gz,
                &SensorsPacket::temp_ag,
                
                &SensorsPacket::h_ax,
                &SensorsPacket::h_ay,
                &SensorsPacket::h_az,
                
                &SensorsPacket::mx,
                &SensorsPacket::my,
                &SensorsPacket::mz,
                &SensorsPacket::temp_m,
                
                &SensorsPacket::baro_temp,
                &SensorsPacket::baro_press,
                
                &SensorsPacket::latitude,
                &SensorsPacket::longitude,
                &SensorsPacket::altitude,
                &SensorsPacket::v_n,
                &SensorsPacket::v_e,
                &SensorsPacket::v_d,
                &SensorsPacket::hAcc,
                &SensorsPacket::vAcc,
                
                &SensorsPacket::sat,
                &SensorsPacket::fix,
                &SensorsPacket::system_status,
                &SensorsPacket::system_time
                

               
            );
            return ret;
        }
        
    public:
        ~SensorsPacket();

        SensorsPacket();
        /**
         * @brief Deserialize Sensors Packet
         * 
         * @param data 
         */
        SensorsPacket(const RnpPacketSerialized& packet);

        /**
         * @brief Serialize Sensors Packet
         * 
         * @param buf 
         */
        void serialize(std::vector<uint8_t>& buf) override;

        
        // accel gyro
        float ax, ay, az;       // acceleration (g's)
        float gx, gy, gz;       // angular rates (deg/s)
        float temp_ag;

        // high g accel
        float h_ax,h_ay,h_az;   // high g accel (g's)
        
        // magnetometer
        float mx, my, mz;       // magnetometer (G)
        float temp_m;
        
        // barometer
        float baro_temp, baro_press;
        
        //gps
        uint32_t latitude, longitude; 
        float altitude;
        float v_n, v_e, v_d;
        float hAcc, vAcc;
        uint8_t sat, fix;

        //system details
        uint32_t system_status;
        uint32_t system_time;

        static constexpr size_t size(){
            return getSerializer().member_size();
        }

};


