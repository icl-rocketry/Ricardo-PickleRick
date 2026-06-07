#pragma once

#include <string>
#include <vector>

#include <librnp/rnp_serializer.h>
#include <unistd.h>

class EstimatorLogframe {
private:
    static constexpr auto getSerializer()
    {
        auto ret = RnpSerializer(
            &EstimatorLogframe::timestamp_us,

            &EstimatorLogframe::raw_ax,
            &EstimatorLogframe::raw_ay,
            &EstimatorLogframe::raw_az,

            &EstimatorLogframe::raw_gx,
            &EstimatorLogframe::raw_gy,
            &EstimatorLogframe::raw_gz,

            &EstimatorLogframe::filtered_ax,
            &EstimatorLogframe::filtered_ay,
            &EstimatorLogframe::filtered_az,

            &EstimatorLogframe::filtered_gx,
            &EstimatorLogframe::filtered_gy,
            &EstimatorLogframe::filtered_gz,

            &EstimatorLogframe::controller_batt_V,
            &EstimatorLogframe::controller_voltage_scale,
            &EstimatorLogframe::controller_thrust_top_cmd,
            &EstimatorLogframe::controller_thrust_bottom_cmd,
            &EstimatorLogframe::controller_fx_cmd,
            &EstimatorLogframe::controller_fx_cmd_outer,
            &EstimatorLogframe::controller_position_control_enabled,
            &EstimatorLogframe::controller_pos_err_x,
            &EstimatorLogframe::controller_pos_err_y,
            &EstimatorLogframe::controller_pos_err_z,
            &EstimatorLogframe::controller_vel_err_x,
            &EstimatorLogframe::controller_vel_err_y,
            &EstimatorLogframe::controller_vel_err_z
        );
        return ret;
    }

public:
    uint64_t timestamp_us;

    float raw_ax, raw_ay, raw_az;
    float raw_gx, raw_gy, raw_gz;
    float filtered_ax, filtered_ay, filtered_az;
    float filtered_gx, filtered_gy, filtered_gz;
    float controller_batt_V;
    float controller_voltage_scale;
    float controller_thrust_top_cmd;
    float controller_thrust_bottom_cmd;
    float controller_fx_cmd;
    float controller_fx_cmd_outer;
    uint8_t controller_position_control_enabled;
    float controller_pos_err_x;
    float controller_pos_err_y;
    float controller_pos_err_z;
    float controller_vel_err_x;
    float controller_vel_err_y;
    float controller_vel_err_z;

    std::string stringify() const
    {
        return getSerializer().stringify(*this) + "\n";
    };

    static std::string csvHeader()
    {
        return "timestamp_us,"
               "raw_ax,raw_ay,raw_az,"
               "raw_gx,raw_gy,raw_gz,"
               "filtered_ax,filtered_ay,filtered_az,"
               "filtered_gx,filtered_gy,filtered_gz,"
               "controller_batt_V,"
               "controller_voltage_scale,"
               "controller_thrust_top_cmd,"
               "controller_thrust_bottom_cmd,"
               "controller_fx_cmd,"
               "controller_fx_cmd_outer,"
               "controller_position_control_enabled,"
               "controller_pos_err_x,"
               "controller_pos_err_y,"
               "controller_pos_err_z,"
               "controller_vel_err_x,"
               "controller_vel_err_y,"
               "controller_vel_err_z,\n";
    }

    std::vector<uint8_t> serialize() const
    {
        return getSerializer().serialize(*this);
    };

    static constexpr size_t size()
    {
        return getSerializer().member_size();
    };
};
