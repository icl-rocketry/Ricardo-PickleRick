#pragma once

namespace DebugConfig
{
    // Currently active debug/performance prints.
    static constexpr bool PerformancePrintEnabled = false;
    static constexpr bool RtkDiffPrintEnabled = false;
    static constexpr bool ControllerFramePrintEnabled = true;

    // Optional debug prints.
    static constexpr bool SerialAlivePrintEnabled = false;
    static constexpr bool RtkRoutingPrintEnabled = false;
    static constexpr bool GpsLatencyPrintEnabled = false;
    static constexpr bool EkfTimingPrintEnabled = false;
    static constexpr bool EkfAttitudeInitPrintEnabled = false;

    // TF-Luna diagnostics.
    static constexpr bool TFLunaInitPrintEnabled = false;
    static constexpr bool TFLunaReadFailPrintEnabled = false;
    static constexpr bool TFLunaReadRecoveredPrintEnabled = false;
    static constexpr bool TFLunaReadOkPrintEnabled = false;
}
