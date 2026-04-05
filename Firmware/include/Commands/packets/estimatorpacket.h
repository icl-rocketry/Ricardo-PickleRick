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

                &EstimatorPacket::acc_n,
                &EstimatorPacket::acc_e,
                &EstimatorPacket::acc_d,

                &EstimatorPacket::q0,
                &EstimatorPacket::q1,
                &EstimatorPacket::q2,
                &EstimatorPacket::q3,

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

                &EstimatorPacket::y_mx,
                &EstimatorPacket::y_my,
                &EstimatorPacket::y_mz,

                &EstimatorPacket::y_ax,
                &EstimatorPacket::y_ay,
                &EstimatorPacket::y_az,

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
        float acc_n, acc_e, acc_d;              // acceleration (m/s^2) (NED frame)

        float q0, q1, q2, q3;                   // quaternions

        float b_gx, b_gy, b_gz;                 // gyro biases (rad/s) (body frame)
        float b_ax, b_ay, b_az;                 // low-g accel biases (m/s^2) (body frame)
        
        uint8_t calibration_quality;            // 0 if no calib, 1 if bias calib, 2 if mag vec calib
        
        float h_mx, h_my, h_mz;                 // expected mag readings (body frame)
        float h_ax, h_ay, h_az;                 // expected accel readings (body frame)
    
        float y_mx, y_my, y_mz;                 // innovation from mag readings (body frame)
        float y_ax, y_ay, y_az;                 // innovation from accel readings (body frame)
        
        float b_hax, b_hay, b_haz;              // high-g accel biases (m/s^2) (body frame)
        float ref_mn, ref_me, ref_md;           // ref mag vec for declination (G) (NED frame)
        
        //system details
        uint32_t system_status;
        uint32_t system_time;

        static constexpr size_t size(){
            return getSerializer().member_size();
        }

};


