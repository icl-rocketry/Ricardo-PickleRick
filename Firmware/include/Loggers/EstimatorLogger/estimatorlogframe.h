#pragma once

#include <string>
#include <vector>

#include <librnp/rnp_serializer.h>

// Estimator analysis log. Field order is deliberately grouped as:
// time/status, raw measurements, estimated state, measurement diagnostics,
// covariance, then controller context.
class EstimatorLogframe {
private:
    static constexpr auto getSerializer()
    {
        return RnpSerializer(
            &EstimatorLogframe::timestamp_us,
            &EstimatorLogframe::gps_utc_hour, &EstimatorLogframe::gps_utc_minute, &EstimatorLogframe::gps_utc_second,
            &EstimatorLogframe::rtk_utc_hour, &EstimatorLogframe::rtk_utc_minute, &EstimatorLogframe::rtk_utc_second,
            &EstimatorLogframe::estimator_state,
            &EstimatorLogframe::raw_ax, &EstimatorLogframe::raw_ay, &EstimatorLogframe::raw_az,
            &EstimatorLogframe::raw_gx, &EstimatorLogframe::raw_gy, &EstimatorLogframe::raw_gz,
            &EstimatorLogframe::filtered_ax, &EstimatorLogframe::filtered_ay, &EstimatorLogframe::filtered_az,
            &EstimatorLogframe::filtered_gx, &EstimatorLogframe::filtered_gy, &EstimatorLogframe::filtered_gz,
            &EstimatorLogframe::raw_hax, &EstimatorLogframe::raw_hay, &EstimatorLogframe::raw_haz,
            &EstimatorLogframe::raw_mx, &EstimatorLogframe::raw_my, &EstimatorLogframe::raw_mz,
            &EstimatorLogframe::raw_baro_temp, &EstimatorLogframe::raw_baro_press,
            &EstimatorLogframe::raw_gps_lat_e7, &EstimatorLogframe::raw_gps_lon_e7, &EstimatorLogframe::raw_gps_alt,
            &EstimatorLogframe::raw_gps_vn, &EstimatorLogframe::raw_gps_ve, &EstimatorLogframe::raw_gps_vd,
            &EstimatorLogframe::raw_gps_hacc, &EstimatorLogframe::raw_gps_vacc,
            &EstimatorLogframe::raw_gps_sat, &EstimatorLogframe::raw_gps_fix, &EstimatorLogframe::raw_gps_valid,
            &EstimatorLogframe::raw_gps_updated, &EstimatorLogframe::raw_gps_timestamp_us,
            &EstimatorLogframe::raw_lidar_dist_cm, &EstimatorLogframe::raw_lidar_amp,
            &EstimatorLogframe::raw_lidar_temp, &EstimatorLogframe::raw_lidar_valid, &EstimatorLogframe::raw_lidar_timestamp_us,
            &EstimatorLogframe::pn, &EstimatorLogframe::pe, &EstimatorLogframe::pd,
            &EstimatorLogframe::vn, &EstimatorLogframe::ve, &EstimatorLogframe::vd,
            &EstimatorLogframe::q0, &EstimatorLogframe::q1, &EstimatorLogframe::q2, &EstimatorLogframe::q3,
            &EstimatorLogframe::roll, &EstimatorLogframe::pitch, &EstimatorLogframe::yaw,
            &EstimatorLogframe::an, &EstimatorLogframe::ae, &EstimatorLogframe::ad,
            &EstimatorLogframe::bgx, &EstimatorLogframe::bgy, &EstimatorLogframe::bgz,
            &EstimatorLogframe::bax, &EstimatorLogframe::bay, &EstimatorLogframe::baz,
            &EstimatorLogframe::gps_pn, &EstimatorLogframe::gps_pe, &EstimatorLogframe::gps_pd,
            &EstimatorLogframe::rtk_delay_us, &EstimatorLogframe::calibration_quality,
            &EstimatorLogframe::h_mx, &EstimatorLogframe::h_my, &EstimatorLogframe::h_mz,
            &EstimatorLogframe::h_ax, &EstimatorLogframe::h_ay, &EstimatorLogframe::h_az,
            &EstimatorLogframe::h_bt, &EstimatorLogframe::h_bp,
            &EstimatorLogframe::h_pn, &EstimatorLogframe::h_pe, &EstimatorLogframe::h_pd,
            &EstimatorLogframe::h_vn, &EstimatorLogframe::h_ve, &EstimatorLogframe::h_vd, &EstimatorLogframe::h_lidar,
            &EstimatorLogframe::y_mx, &EstimatorLogframe::y_my, &EstimatorLogframe::y_mz,
            &EstimatorLogframe::y_ax, &EstimatorLogframe::y_ay, &EstimatorLogframe::y_az,
            &EstimatorLogframe::y_bt, &EstimatorLogframe::y_bp,
            &EstimatorLogframe::y_pn, &EstimatorLogframe::y_pe, &EstimatorLogframe::y_pd,
            &EstimatorLogframe::y_vn, &EstimatorLogframe::y_ve, &EstimatorLogframe::y_vd, &EstimatorLogframe::y_lidar,
            &EstimatorLogframe::nis_mag, &EstimatorLogframe::nis_accel, &EstimatorLogframe::nis_baro,
            &EstimatorLogframe::nis_gps, &EstimatorLogframe::nis_rtk, &EstimatorLogframe::nis_lidar,
            &EstimatorLogframe::p0, &EstimatorLogframe::p1, &EstimatorLogframe::p2, &EstimatorLogframe::p3,
            &EstimatorLogframe::p4, &EstimatorLogframe::p5, &EstimatorLogframe::p6, &EstimatorLogframe::p7,
            &EstimatorLogframe::p8, &EstimatorLogframe::p9, &EstimatorLogframe::p10, &EstimatorLogframe::p11,
            &EstimatorLogframe::p12, &EstimatorLogframe::p13, &EstimatorLogframe::p14, &EstimatorLogframe::p15,
            &EstimatorLogframe::controller_batt_V, &EstimatorLogframe::controller_voltage_scale,
            &EstimatorLogframe::controller_thrust_top_cmd, &EstimatorLogframe::controller_thrust_bottom_cmd,
            &EstimatorLogframe::controller_fx_cmd, &EstimatorLogframe::controller_fx_cmd_outer,
            &EstimatorLogframe::controller_position_control_enabled,
            &EstimatorLogframe::controller_pos_err_x, &EstimatorLogframe::controller_pos_err_y, &EstimatorLogframe::controller_pos_err_z,
            &EstimatorLogframe::controller_vel_err_x, &EstimatorLogframe::controller_vel_err_y, &EstimatorLogframe::controller_vel_err_z
        );
    }

public:
    uint64_t timestamp_us;
    uint8_t gps_utc_hour, gps_utc_minute;
    float gps_utc_second;
    uint8_t rtk_utc_hour, rtk_utc_minute;
    float rtk_utc_second;
    uint8_t estimator_state;
    float raw_ax, raw_ay, raw_az, raw_gx, raw_gy, raw_gz;
    float filtered_ax, filtered_ay, filtered_az, filtered_gx, filtered_gy, filtered_gz;
    float raw_hax, raw_hay, raw_haz, raw_mx, raw_my, raw_mz;
    float raw_baro_temp, raw_baro_press;
    int32_t raw_gps_lat_e7, raw_gps_lon_e7;
    float raw_gps_alt, raw_gps_vn, raw_gps_ve, raw_gps_vd, raw_gps_hacc, raw_gps_vacc;
    uint8_t raw_gps_sat, raw_gps_fix, raw_gps_valid, raw_gps_updated;
    uint32_t raw_gps_timestamp_us;
    uint16_t raw_lidar_dist_cm, raw_lidar_amp;
    float raw_lidar_temp;
    uint8_t raw_lidar_valid;
    uint32_t raw_lidar_timestamp_us;
    float pn, pe, pd, vn, ve, vd, q0, q1, q2, q3, roll, pitch, yaw, an, ae, ad;
    float bgx, bgy, bgz, bax, bay, baz, gps_pn, gps_pe, gps_pd;
    uint32_t rtk_delay_us;
    uint8_t calibration_quality;
    float h_mx, h_my, h_mz, h_ax, h_ay, h_az, h_bt, h_bp;
    float h_pn, h_pe, h_pd, h_vn, h_ve, h_vd, h_lidar;
    float y_mx, y_my, y_mz, y_ax, y_ay, y_az, y_bt, y_bp;
    float y_pn, y_pe, y_pd, y_vn, y_ve, y_vd, y_lidar;
    float nis_mag, nis_accel, nis_baro, nis_gps, nis_rtk, nis_lidar;
    float p0, p1, p2, p3, p4, p5, p6, p7, p8, p9, p10, p11, p12, p13, p14, p15;
    float controller_batt_V, controller_voltage_scale;
    float controller_thrust_top_cmd, controller_thrust_bottom_cmd;
    float controller_fx_cmd, controller_fx_cmd_outer;
    uint8_t controller_position_control_enabled;
    float controller_pos_err_x, controller_pos_err_y, controller_pos_err_z;
    float controller_vel_err_x, controller_vel_err_y, controller_vel_err_z;

    std::string stringify() const { return getSerializer().stringify(*this) + "\n"; }

    static std::string csvHeader()
    {
        return "timestamp_us,gps_utc_hour,gps_utc_minute,gps_utc_second,"
               "rtk_utc_hour,rtk_utc_minute,rtk_utc_second,estimator_state,"
               "raw_ax,raw_ay,raw_az,raw_gx,raw_gy,raw_gz,filtered_ax,filtered_ay,filtered_az,filtered_gx,filtered_gy,filtered_gz,"
               "raw_hax,raw_hay,raw_haz,raw_mx,raw_my,raw_mz,raw_baro_temp,raw_baro_press,"
               "raw_gps_lat_e7,raw_gps_lon_e7,raw_gps_alt,raw_gps_vn,raw_gps_ve,raw_gps_vd,raw_gps_hacc,raw_gps_vacc,"
               "raw_gps_sat,raw_gps_fix,raw_gps_valid,raw_gps_updated,raw_gps_timestamp_us,"
               "raw_lidar_dist_cm,raw_lidar_amp,raw_lidar_temp,raw_lidar_valid,raw_lidar_timestamp_us,"
               "pn,pe,pd,vn,ve,vd,q0,q1,q2,q3,roll,pitch,yaw,an,ae,ad,bgx,bgy,bgz,bax,bay,baz,gps_pn,gps_pe,gps_pd,rtk_delay_us,calibration_quality,"
               "h_mx,h_my,h_mz,h_ax,h_ay,h_az,h_bt,h_bp,h_pn,h_pe,h_pd,h_vn,h_ve,h_vd,h_lidar,"
               "y_mx,y_my,y_mz,y_ax,y_ay,y_az,y_bt,y_bp,y_pn,y_pe,y_pd,y_vn,y_ve,y_vd,y_lidar,"
               "nis_mag,nis_accel,nis_baro,nis_gps,nis_rtk,nis_lidar,"
               "P_pn,P_pe,P_pd,P_vn,P_ve,P_vd,P_q0,P_q1,P_q2,P_q3,P_bax,P_bay,P_baz,P_bgx,P_bgy,P_bgz,"
               "controller_batt_V,controller_voltage_scale,controller_thrust_top_cmd,controller_thrust_bottom_cmd,"
               "controller_fx_cmd,controller_fx_cmd_outer,controller_position_control_enabled,"
               "controller_pos_err_x,controller_pos_err_y,controller_pos_err_z,controller_vel_err_x,controller_vel_err_y,controller_vel_err_z\n";
    }

    std::vector<uint8_t> serialize() const { return getSerializer().serialize(*this); }
    static constexpr size_t size() { return getSerializer().member_size(); }
};
