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
    constexpr uint32_t hard_abort_hold_ms = 100 * 1000; // 100 seconds
    
    constexpr float winch_line_rate_fwd_mps = 0.05; // m/s
    constexpr float winch_line_rate_rev_mps = 0.06; // m/s

    constexpr float    debug_speed       = 0.25f;  // 25% of the linear range
    constexpr uint32_t debug_fwd_ms      = 3000;
    constexpr uint32_t debug_pause_ms    = 2000;
    constexpr uint32_t debug_rev_ms      = 3000;

}