#pragma once

#include <cstdint>

namespace LandingConfig {

inline constexpr float CutoffHeightAboveGroundM = 0.1f; 
inline constexpr float DescentRateMps = 0.1f;
inline constexpr float MinDescentDurationS = 1.0f;
inline constexpr float DescentAccelFraction = 0.2f;
inline constexpr float CutoffHeightToleranceM = 0.08f;
inline constexpr uint32_t ThrottleFadeDurationMs = 1200;

} // namespace LandingConfig
