#pragma once

#include <librnp/rnp_packet.h>
#include <librnp/rnp_serializer.h>

#include <vector>

class ToasterTelem : public RnpPacket{
private:
//serializer framework
    static constexpr auto getSerializer()
    {
        auto ret = RnpSerializer(
            &ToasterTelem::upperEndtopPressed,
            &ToasterTelem::lowerEndtopPressed,
            &ToasterTelem::stepperEnabled,
            &ToasterTelem::act0Position,
            &ToasterTelem::act1Position,
            &ToasterTelem::act2Position,
            &ToasterTelem::systemStatus,
            &ToasterTelem::systemTime
        );

        return ret;
    }

public:
    ~ToasterTelem();

    ToasterTelem();
    /**
     * @brief Deserialize Telemetry Packet
     *
     * @param data
     */
    ToasterTelem(const RnpPacketSerialized& packet);

    /**
     * @brief Serialize Telemetry Packet
     *
     * @param buf
     */
    void serialize(std::vector<uint8_t>& buf) override;

    // Actuator telem
    bool upperEndtopPressed;
    bool lowerEndtopPressed;
    bool stepperEnabled;
    uint16_t act0Position, act1Position, act2Position;

    // System telem
    uint32_t systemStatus;
    uint64_t systemTime;

    static constexpr size_t size(){
        return getSerializer().member_size();
    }
};

/*
{
  "task_name": "toaster_telemetry",
  "autostart": false,
  "poll_delta": 500,
  "running": false,
  "logger": true,
  "receiveOnly": false,
  "request_config": {
    "source": 1,
    "destination": <toaster-addr>,
    "destination_service": 10,
    "command_id": 8,
    "command_arg": 0
  },
  "packet_descriptor": {
    "upperEndtopPressed": "bool",
    "lowerEndtopPressed": "bool",
    "stepperEnabled": "bool",
    "act0Position": "uint16_t",
    "act1Position": "uint16_t",
    "act2Position": "uint16_t",
    "system_status": "uint32_t",
    "system_time": "uint64_t"
  },
  "bitfield_decoders": [
    {
      "variable_name": "state",
      "bitfield": "system_status",
      "flags": [
        {
            "id": 0,
            "description": "STATE_ZERO"
        }
        {
            "id": 1,
            "description": "STATE_DEFAULT"
        }
        {
            "id": 2,
            "description": "STATE_ARMED"
        }
        {
            "id": 3,
            "description": "STATE_DEPLOY"
        }
        {
            "id": 4,
            "description": "STATE_COMMAND"
        }
        {
            "id": 10,
            "description": "ERROR_ZERO_TIMEOUT"
        }
      ]
    }
  ],
  "rxCounter": 0,
  "txCounter": 0,
  "connected": true,
  "lastReceivedPacket": "",
  "rxBytes": 0,
  "txBytes": 0,
  "groups": []
}
*/

class PIDTelem : public RnpPacket{
private:
//serializer framework
    static constexpr auto getSerializer()
    {
        auto ret = RnpSerializer(
            &PIDTelem::measurement,
            &PIDTelem::target,
            &PIDTelem::error,
            &PIDTelem::kp,
            &PIDTelem::ki,
            &PIDTelem::kd,
            &PIDTelem::control,
            &PIDTelem::time
        );

        return ret;
    }

public:
    ~PIDTelem();

    PIDTelem();
    /**
     * @brief Deserialize Telemetry Packet
     *
     * @param data
     */
    PIDTelem(const RnpPacketSerialized& packet);

    /**
     * @brief Serialize Telemetry Packet
     *
     * @param buf
     */
    void serialize(std::vector<uint8_t>& buf) override;

    // Actuator telem
    double measurement, target, error, kp, ki, kd, control;
    uint64_t time;

    static constexpr size_t size(){
        return getSerializer().member_size();
    }
};

/*
{
  "task_name": "pid_telemetry",
  "autostart": false,
  "poll_delta": 500,
  "running": false,
  "logger": true,
  "receiveOnly": false,
  "request_config": {
    "source": 1,
    "destination": <toaster-addr>,
    "destination_service": 10,
    "command_id": 9,
    "command_arg": 0
  },
  "packet_descriptor": {
    "measurement": "double",
    "target": "double",
    "error": "double",
    "kp": "double",
    "ki": "double",
    "kd": "double",
    "control": "double",
    "time": "uint64_t"
  },
  "rxCounter": 0,
  "txCounter": 0,
  "connected": true,
  "lastReceivedPacket": "",
  "rxBytes": 0,
  "txBytes": 0,
  "groups": []
}
*/