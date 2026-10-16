namespace GuidedConfig
{
    constexpr float guided_alt_start = 950; // 950m (assuming cut off at 1000m)
    constexpr float max_differential = 6.5; // cm
    constexpr int guided_zero_speed_angle = 500;
    constexpr float guided_min_speed = 0.01; // deadband to avoid jitter
    constexpr int winch_stop_us     = 1500;
    constexpr int winch_fwd_edge_us = 1550;   // deadzone edge
    constexpr int winch_fwd_max_us  = 1950;
    constexpr int winch_rev_edge_us = 1450;
    constexpr int winch_rev_max_us  = 950;
}