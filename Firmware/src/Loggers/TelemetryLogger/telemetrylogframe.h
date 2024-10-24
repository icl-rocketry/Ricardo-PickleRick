#pragma once

#include <librnp/rnp_serializer.h>
#include <unistd.h>

class TelemetryLogframe{
private:  
    static constexpr auto getSerializer()
    {
        auto ret = RnpSerializer(
            &TelemetryLogframe::gps_long,
            &TelemetryLogframe::gps_lat,
            &TelemetryLogframe::gps_alt,
            &TelemetryLogframe::gps_v_n,
            &TelemetryLogframe::gps_v_e,
            &TelemetryLogframe::gps_v_d,
            &TelemetryLogframe::gps_sat,
            &TelemetryLogframe::gps_fix,
            &TelemetryLogframe::ax,
            &TelemetryLogframe::ay,
            &TelemetryLogframe::az,
            &TelemetryLogframe::h_ax,
            &TelemetryLogframe::h_ay,
            &TelemetryLogframe::h_az,
            &TelemetryLogframe::gx,
            &TelemetryLogframe::gy,
            &TelemetryLogframe::gz,
            &TelemetryLogframe::mx,
            &TelemetryLogframe::my,
            &TelemetryLogframe::mz,
            &TelemetryLogframe::imu_temp,
            &TelemetryLogframe::baro_alt,
            &TelemetryLogframe::baro_temp,
            &TelemetryLogframe::baro_press,
            &TelemetryLogframe::logic_voltage,
            &TelemetryLogframe::logic_percent,
            &TelemetryLogframe::dep_voltage,
            &TelemetryLogframe::dep_current,
            &TelemetryLogframe::roll,
            &TelemetryLogframe::pitch,
            &TelemetryLogframe::yaw,
            &TelemetryLogframe::q0,
            &TelemetryLogframe::q1,
            &TelemetryLogframe::q2,
            &TelemetryLogframe::q3,
            &TelemetryLogframe::pn,
            &TelemetryLogframe::pe,
            &TelemetryLogframe::pd,
            &TelemetryLogframe::vn,
            &TelemetryLogframe::ve,
            &TelemetryLogframe::vd,
            &TelemetryLogframe::an,
            &TelemetryLogframe::ae,
            &TelemetryLogframe::ad,
            &TelemetryLogframe::rssi,
            &TelemetryLogframe::packet_rssi,
            &TelemetryLogframe::snr,
            &TelemetryLogframe::packet_snr,
            &TelemetryLogframe::timestamp
            
           

        );
        return ret;
    }

public:
    //gps
    float gps_long, gps_lat;
    long gps_alt;
    long gps_v_n, gps_v_e, gps_v_d;
    uint8_t gps_sat, gps_fix; 
    //imu
    float ax, ay, az;
    float h_ax, h_ay, h_az;
    float gx, gy, gz;
    float mx, my, mz;
    float imu_temp;
    //baro
    float baro_alt,baro_temp,baro_press;
    //battery
    int logic_voltage;
    int logic_percent;
    int dep_voltage;
    int dep_current;
    //orientation
    float roll, pitch, yaw;
    float q0,q1,q2,q3;
    //position
    float pn,pe,pd;
    //velocity
    float vn,ve,vd;
    //linear acceleration
    float an,ae,ad;
    //radio details
    int16_t rssi, packet_rssi;
    float snr, packet_snr;

    uint64_t timestamp;

    std::string stringify()const{
        return getSerializer().stringify(*this) + "\n";
    };

};
