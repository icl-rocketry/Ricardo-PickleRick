#pragma once

#include <librnp/rnp_packet.h>
#include <librnp/rnp_serializer.h>

#include <vector>

class EstimatorPacket : public RnpPacket{
    private:
    //serializer framework
        static constexpr auto getSerializer()
        {
            auto ret = RnpSerializer(
                &EstimatorPacket::pos_n,
                &EstimatorPacket::pos_e,
                &EstimatorPacket::pos_d,

                &EstimatorPacket::vel_n,
                &EstimatorPacket::vel_e,
                &EstimatorPacket::vel_d,
                
                &EstimatorPacket::q0,
                &EstimatorPacket::q1,
                &EstimatorPacket::q2,
                &EstimatorPacket::q3,

                &EstimatorPacket::gps_pos_n,
                &EstimatorPacket::gps_pos_e,
                &EstimatorPacket::gps_pos_d,

                &EstimatorPacket::b_gx,
                &EstimatorPacket::b_gy,
                &EstimatorPacket::b_gz,

                &EstimatorPacket::b_ax,
                &EstimatorPacket::b_ay,
                &EstimatorPacket::b_az,

                &EstimatorPacket::calibration_quality,

                // Stuff for dev/debugging

                &EstimatorPacket::h_mx,
                &EstimatorPacket::h_my,
                &EstimatorPacket::h_mz,

                &EstimatorPacket::h_ax,
                &EstimatorPacket::h_ay,
                &EstimatorPacket::h_az,

                &EstimatorPacket::h_bt,
                &EstimatorPacket::h_bp,
                
                &EstimatorPacket::h_pn,
                &EstimatorPacket::h_pe,
                &EstimatorPacket::h_pd,

                &EstimatorPacket::h_vn,
                &EstimatorPacket::h_ve,
                &EstimatorPacket::h_vd,

                &EstimatorPacket::y_mx,
                &EstimatorPacket::y_my,
                &EstimatorPacket::y_mz,

                &EstimatorPacket::y_ax,
                &EstimatorPacket::y_ay,
                &EstimatorPacket::y_az,

                &EstimatorPacket::y_bt,
                &EstimatorPacket::y_bp,

                &EstimatorPacket::y_pn,
                &EstimatorPacket::y_pe,
                &EstimatorPacket::y_pd,

                &EstimatorPacket::y_vn,
                &EstimatorPacket::y_ve,
                &EstimatorPacket::y_vd,

                &EstimatorPacket::b_hax,
                &EstimatorPacket::b_hay,
                &EstimatorPacket::b_haz,

                &EstimatorPacket::ref_mn,
                &EstimatorPacket::ref_me,
                &EstimatorPacket::ref_md,
                
                &EstimatorPacket::system_status,
                &EstimatorPacket::system_time
                

               
            );
            return ret;
        }
        
    public:
        ~EstimatorPacket();

        EstimatorPacket();
        /**
         * @brief Deserialize Estimator Packet
         * 
         * @param data 
         */
        EstimatorPacket(const RnpPacketSerialized& packet);

        /**
         * @brief Serialize Estimator Packet
         * 
         * @param buf 
         */
        void serialize(std::vector<uint8_t>& buf) override;

        
        // accel gyro
        float pos_n, pos_e, pos_d;              // position (m) (NED frame)
        float vel_n, vel_e, vel_d;              // velocity (m/s) (NED frame)

        float q0, q1, q2, q3;                   // quaternions

        float b_gx, b_gy, b_gz;                 // gyro biases (rad/s) (body frame)
        float b_ax, b_ay, b_az;                 // low-g accel biases (m/s^2) (body frame)
        
        float gps_pos_n, gps_pos_e, gps_pos_d;  // position (m) (NED frame)

        uint8_t calibration_quality;            // 0 if no calib, 1 if bias calib, 2 if mag vec calib
        
        float h_mx, h_my, h_mz;                 // expected mag readings (body frame)
        float h_ax, h_ay, h_az;                 // expected accel readings (body frame)
        float h_bt, h_bp;                       // expected baro readings (NED frame)
        float h_pn, h_pe, h_pd;                 // expected gps pos readings (NED frame)
        float h_vn, h_ve, h_vd;                 // expected gps vel readings (NED frame)
    
        float y_mx, y_my, y_mz;                 // innovation from mag readings (body frame)
        float y_ax, y_ay, y_az;                 // innovation from accel readings (body frame)
        float y_bt, y_bp;                       // innovation from baro readings (NED frame)
        float y_pn, y_pe, y_pd;                 // innovation from gps pos readings (NED frame)
        float y_vn, y_ve, y_vd;                 // innovation from gps vel readings (NED frame)
        
        float b_hax, b_hay, b_haz;              // high-g accel biases (m/s^2) (body frame)
        float ref_mn, ref_me, ref_md;           // ref mag vec for declination (G) (NED frame)
        
        //system details
        uint32_t system_status;
        uint32_t system_time;

        static constexpr size_t size(){
            return getSerializer().member_size();
        }

};


