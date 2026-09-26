#pragma once

enum class AngleWrapperConfig {
    RADIANS,
    DEGREES
};

/**
 * @brief Stateful angle wrapper util class.
 *
 * This class smooths continuous angle readings over the discontinuous
 * boundary, i.e. if you have 3 angle readings, [ 352, 359, 1 ], this will be
 * smoothed to [ 352, 359, 361 ].
 *
 * This class will also wrap negative values so be sure that is accounted for.
 *
 * @tparam config Whether the class is setup for Degree readings or Radian
 * readings
 */
template<AngleWrapperConfig config>
class AngleWrapper {
public:
    AngleWrapper();

    double update(double angle);
};

