#pragma once

#include <librnp/rnp_packet.h>
#include <librnp/rnp_serializer.h>

class TelemetryPacket : public RnpPacket{
    private:
    //serializer framework
        static constexpr auto getSerializer()
        {
            auto ret = RnpSerializer(
                &TelemetryPacket::pn,
                &TelemetryPacket::pe,
                &TelemetryPacket::pd,
                &TelemetryPacket::vn,
                &TelemetryPacket::ve,
                &TelemetryPacket::vd,
                &TelemetryPacket::q0,
                &TelemetryPacket::q1,
                &TelemetryPacket::q2,
                &TelemetryPacket::q3,
                &TelemetryPacket::roll,
                &TelemetryPacket::pitch,
                &TelemetryPacket::yaw,
                &TelemetryPacket::rocket_q0,
                &TelemetryPacket::rocket_q1,
                &TelemetryPacket::rocket_q2,
                &TelemetryPacket::rocket_q3,
                &TelemetryPacket::rocket_roll,
                &TelemetryPacket::rocket_pitch,
                &TelemetryPacket::rocket_yaw,
                &TelemetryPacket::latitude,
                &TelemetryPacket::longitude,
                &TelemetryPacket::altitude,
                &TelemetryPacket::sat,
                &TelemetryPacket::ax,
                &TelemetryPacket::ay,
                &TelemetryPacket::az,
                &TelemetryPacket::h_ax,
                &TelemetryPacket::h_ay,
                &TelemetryPacket::h_az,
                &TelemetryPacket::gx,
                &TelemetryPacket::gy,
                &TelemetryPacket::gz,
                &TelemetryPacket::mx,
                &TelemetryPacket::my,
                &TelemetryPacket::mz,
                &TelemetryPacket::baro_temp,
                &TelemetryPacket::baro_press,
                &TelemetryPacket::pdb_batt_mV,
                &TelemetryPacket::pdb_batt_fresh,
                &TelemetryPacket::system_status,
                &TelemetryPacket::system_time,
                &TelemetryPacket::rssi,
                &TelemetryPacket::snr,
                &TelemetryPacket::lidar_dist,
                &TelemetryPacket::lidar_amp,
                &TelemetryPacket::lidar_temp,
                &TelemetryPacket::rtk_x,
                &TelemetryPacket::rtk_y,
                &TelemetryPacket::rtk_z,
                &TelemetryPacket::rtk_u,
                &TelemetryPacket::rtk_v,
                &TelemetryPacket::rtk_w,
                &TelemetryPacket::rtk_fix_quality,
                &TelemetryPacket::rtk_valid,
                &TelemetryPacket::rtk_timestamp_us
            );
            return ret;
        }
        
    public:
        ~TelemetryPacket();

        TelemetryPacket();
        /**
         * @brief Deserialize Telemetry Packet
         * 
         * @param data 
         */
        TelemetryPacket(const RnpPacketSerialized& packet);

        /**
         * @brief Serialize Telemetry Packet
         * 
         * @param buf 
         */
        void serialize(std::vector<uint8_t>& buf) override;

        
        //packet header
        //PacketHeader header{static_cast<uint8_t>(packet::TELEMETRY), packet_size()};
        //estimator output
        float pn, pe, pd; // position NED (m)
        float vn, ve, vd; // velocity NED (m/s)
        float an, ae, ad; // acceleration NED (g's)
        //orientation
        float roll,pitch,yaw; // orientation degrees
        float rocket_roll,rocket_pitch,rocket_yaw; // orientation degrees
        float q0,q1,q2,q3; //quaternion representation
        float rocket_q0,rocket_q1,rocket_q2,rocket_q3; //quaternion representation
        //gps
        int32_t latitude,longitude;
        float altitude;
        uint8_t sat;
        //imu
        float ax, ay, az; // acceleration (g's)
        float h_ax,h_ay,h_az;// high g accel (g's)
        float gx, gy, gz; // angular rates (deg/s)
        float mx, my, mz;// magnetometer (uT)
        //barometer
        float baro_temp, baro_press;
        uint16_t pdb_batt_mV;
        uint8_t pdb_batt_fresh;
        //system details
        uint32_t system_status;
        uint64_t system_time;
        //radio details
        int16_t rssi;
        float snr;
        //lidar
        uint16_t lidar_dist;  // cm
        uint16_t lidar_amp;   // signal strength
        float    lidar_temp;  // degrees Celsius

        //rtk
        float rtk_x, rtk_y, rtk_z; // position NED (m)
        float rtk_u, rtk_v, rtk_w; // velocity NED (m/s)
        uint8_t rtk_fix_quality;
        uint8_t rtk_valid;
        uint32_t rtk_timestamp_us;


        static constexpr size_t size(){
            return getSerializer().member_size();
        }

};

