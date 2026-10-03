#pragma once

#include <librnp/rnp_serializer.h>
#include <unistd.h>

class ApogeeLogframe{
private:  
    static constexpr auto getSerializer()
    {
        auto ret = RnpSerializer(
            &ApogeeLogframe::coeff_0,
            &ApogeeLogframe::coeff_1,
            &ApogeeLogframe::coeff_2,
            &ApogeeLogframe::predicted_time,
            &ApogeeLogframe::predicted_altitude,
            &ApogeeLogframe::altitude,
            &ApogeeLogframe::altitude_error,
            &ApogeeLogframe::mlock,
            &ApogeeLogframe::reached,
            &ApogeeLogframe::timestamp
        );
        return ret;
    }

public:
    float coeff_0, coeff_1, coeff_2;
    uint32_t predicted_time;
    float predicted_altitude;
    float altitude;
    float altitude_error;
    uint8_t mlock;
    uint8_t reached;

    uint64_t timestamp;

    std::string stringify()const{
        return getSerializer().stringify(*this) + "\n";
    };

};
