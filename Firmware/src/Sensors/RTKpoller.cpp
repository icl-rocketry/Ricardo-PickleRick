#include "Sensors/RTKpoller.h"
#include "Config/debug_config.h"
#include "Config/timing_config.h"

void RTKPoller::setup()
{
    x_input = 0.0f;
    y_input = 0.0f;
    z_input = 0.0f;
    u_input = 0.0f;
    v_input = 0.0f;
    w_input = 0.0f;
    fix_quality = 0;
    wifi_connected = false;
    gnss_time_of_day_ms = 0;
    m_homeSet = false;
    m_valid = false;
    m_timestamp_us = 0;
    m_prevRtkDebugPosition = Eigen::Vector3f(0.0f, 0.0f, 0.0f);
    m_rtkDebugDiffSum = 0.0f;
    m_rtkDebugDiffIndex = 0;
    m_rtkDebugDiffCount = 0;
    m_havePrevRtkDebugPosition = false;
    for (uint8_t i = 0; i < RTK_DEBUG_DIFF_WINDOW_SIZE; i++) {
        m_rtkDebugDiffWindow[i] = 0.0f;
    }
}

void RTKPoller::update(SensorStructs::RTK_t &data)
{
    const uint32_t now_us = micros();
    const bool fresh_measurement = hasFreshMeasurement(now_us);
    const Eigen::Vector3f position = getPosition();
    data.x = position.x();
    data.y = position.y();
    data.z = position.z();
    data.u = u_input;
    data.v = v_input;
    data.w = w_input;
    data.fix_quality = fix_quality;
    data.wifi_connected = wifi_connected;
    data.valid = fresh_measurement;
    data.home_set = m_homeSet;
    data.gnss_time_of_day_ms = gnss_time_of_day_ms;
    data.measurement_timestamp_us = 0;
    data.timestamp_us = m_timestamp_us;
    data.delay_us = 0;
}

bool RTKPoller::hasMeasurement() const
{
    return hasFreshMeasurement(micros());
}

bool RTKPoller::hasFix() const
{
    return hasMeasurement() && fix_quality != 0;
}

bool RTKPoller::hasFixed() const
{
    return hasMeasurement() && fix_quality == 4;
}

bool RTKPoller::hasFreshMeasurement(const uint32_t now_us) const
{
    return m_valid &&
           m_timestamp_us != 0 &&
           now_us - m_timestamp_us <= TimingConfig::EKF::RTK_CORRECTION_MAX_AGE_US;
}

std::function<void(packetptr_t)> RTKPoller::getThisNetworkCallback()
{
    return [this](packetptr_t packetptr)
    { handlecommand(std::move(packetptr)); };

};

Eigen::Vector3f RTKPoller::getPosition() {
    if (m_homeSet) {
        return Eigen::Vector3f(x_input-x_home, y_input-y_home, z_input-z_home);
    } else {
        return Eigen::Vector3f(x_input, y_input, z_input);
    };
}

Eigen::Vector3f RTKPoller::getPositionRaw() {
    return Eigen::Vector3f(x_input, y_input, z_input);
}

void RTKPoller::handlecommand(packetptr_t packetptr)
{
    std::vector<uint8_t> serializedData = packetptr->getBody();

    if (serializedData.size() != RTKPacket::size()) {
        return;
    }

    rtkdata.deserializeBody(serializedData);
    x_input = rtkdata.x_input;
    y_input = rtkdata.y_input;
    z_input = rtkdata.z_input;
    u_input = rtkdata.u_input;
    v_input = rtkdata.v_input;
    w_input = rtkdata.w_input;
    fix_quality = rtkdata.fix_quality;
    wifi_connected = rtkdata.wifi_connected != 0;
    gnss_time_of_day_ms = rtkdata.gnss_time_of_day_ms;
    m_timestamp_us = micros();
    m_valid = true;

    if constexpr (DebugConfig::RtkDiffPrintEnabled) {
        const Eigen::Vector3f current_position(x_input, y_input, z_input);
        if (m_havePrevRtkDebugPosition) {
            const Eigen::Vector3f delta = current_position - m_prevRtkDebugPosition;
            const Eigen::Vector3f abs_delta = delta.cwiseAbs();
            const float diff_m = delta.norm();

            if (m_rtkDebugDiffCount < RTK_DEBUG_DIFF_WINDOW_SIZE) {
                m_rtkDebugDiffCount++;
            } else {
                m_rtkDebugDiffSum -= m_rtkDebugDiffWindow[m_rtkDebugDiffIndex];
            }

            m_rtkDebugDiffWindow[m_rtkDebugDiffIndex] = diff_m;
            m_rtkDebugDiffSum += diff_m;
            m_rtkDebugDiffIndex = (m_rtkDebugDiffIndex + 1) % RTK_DEBUG_DIFF_WINDOW_SIZE;

            const float avg_diff_m = m_rtkDebugDiffSum / static_cast<float>(m_rtkDebugDiffCount);
            const float diff_cm = diff_m * 100.0f;
            const float avg_diff_cm = avg_diff_m * 100.0f;
            const Eigen::Vector3f abs_delta_cm = abs_delta * 100.0f;
            Serial.printf(
                "RTK DEBUG diff_cm=%.1f avg5_diff_cm=%.1f abs_dx_cm=%.1f abs_dy_cm=%.1f abs_dz_cm=%.1f fix=%u\n",
                diff_cm,
                avg_diff_cm,
                abs_delta_cm.x(),
                abs_delta_cm.y(),
                abs_delta_cm.z(),
                fix_quality
            );
        }
        m_prevRtkDebugPosition = current_position;
        m_havePrevRtkDebugPosition = true;
    }
}
