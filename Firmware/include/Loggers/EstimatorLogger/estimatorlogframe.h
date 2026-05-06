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
            &EstimatorLogframe::filtered_gz
        );
        return ret;
    }

public:
    uint64_t timestamp_us;

    float raw_ax, raw_ay, raw_az;
    float raw_gx, raw_gy, raw_gz;
    float filtered_ax, filtered_ay, filtered_az;
    float filtered_gx, filtered_gy, filtered_gz;

    std::string stringify() const
    {
        return getSerializer().stringify(*this) + "\n";
    };

    std::vector<uint8_t> serialize() const
    {
        return getSerializer().serialize(*this);
    };

    static constexpr size_t size()
    {
        return getSerializer().member_size();
    };
};
