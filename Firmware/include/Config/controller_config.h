#pragma once

namespace ControllerConfig {

// When false, vertical control ignores trajectory acceleration feedforward
// and uses position/velocity feedback plus gravity compensation.
inline constexpr bool VerticalAccelerationFeedforwardEnabled = false;

} // namespace ControllerConfig
