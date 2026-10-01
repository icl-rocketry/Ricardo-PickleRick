#include "toaster.h"

#include "Config/general_config.h"

#include "States/default.h"
#include "States/armed.h"
#include "States/deploy.h"
#include "States/command.h"
#include "States/zero.h"

void Toaster::setup() {
    // Start in default status
    m_toasterMachine.initalize(std::make_unique<ToasterDefault>(m_system, m_toasterStatus));

    // Initialise pins

    pinMode(m_enablePin, OUTPUT);
    stepperDisable();

    pinMode(m_endstopLower, INPUT);
    pinMode(m_endstopUpper, INPUT);

    m_networkmanager.registerService(static_cast<uint8_t>(Services::ID::TOASTER), this->getThisNetworkCallback());
}

void Toaster::update() {
    m_toasterMachine.update();
}

void Toaster::apogeeDetected() {
    // Ensure module is armed
    if (m_toasterMachine.getCurrentStateID() != TOASTER_FLAGS::STATE_ARMED) {
        RicCoreLogging::log<RicCoreLoggingConfig::LOGGERS::SYS>("Toaster not armed, cant deploy.");
        return;
    }

    // Transition into deployed state.
    m_toasterMachine.changeState(std::make_unique<Deploy>(m_system, m_toasterStatus));
}

void Toaster::stepperCommand(const StepperCommandPayload& command) {
    SimpleCommandPacket stepperPacket(static_cast<uint32_t>(NRCPacket::NRC_COMMAND_ID::EXECUTE), static_cast<int32_t>(command));
    stepperPacket.header.source = m_system.networkmanager.getAddress();
    stepperPacket.header.source_service = static_cast<uint8_t>(Services::ID::TOASTER);

    stepperPacket.header.destination_service = 20;
    stepperPacket.header.destination = static_cast<uint8_t>(GeneralConfig::NetworkConfig::CHAD_MASTER);

    m_system.networkmanager.sendPacket(stepperPacket);
}

void Toaster::stepperEnable() {
    // Enable pin is active low
    m_stepperEnabled = true;
    digitalWrite(m_enablePin, LOW);
}

void Toaster::stepperDisable() {
    // Enable pin is active low
    m_stepperEnabled = false;
    digitalWrite(m_enablePin, HIGH);
}

void Toaster::actuatorZero() {
    actuatorPayload.act0Deg = GeneralConfig::act0ZeroAngle;
    actuatorPayload.act1Deg = GeneralConfig::act1ZeroAngle;
    actuatorPayload.act2Deg = GeneralConfig::act2ZeroAngle;

    actuatorCommand(actuatorPayload);
}

void Toaster::actuatorCommand(const ActuatorCommandPayload& command) {
    SimpleCommandPacket actuatorPacket(static_cast<command_t>(NRCPacket::NRC_COMMAND_ID::EXECUTE), 0);

    actuatorPacket.header.source = m_system.networkmanager.getAddress();
    actuatorPacket.header.source_service = static_cast<uint8_t>(Services::ID::TOASTER);

    actuatorPacket.header.destination_service = 10;
    actuatorPacket.header.destination = static_cast<uint8_t>(GeneralConfig::NetworkConfig::CHAD_MASTER);

    actuatorPacket.arg = command.act0Deg;
    m_system.networkmanager.sendPacket(actuatorPacket);

    actuatorPacket.header.destination_service = 11;

    actuatorPacket.arg = command.act1Deg;
    m_system.networkmanager.sendPacket(actuatorPacket);

    actuatorPacket.header.destination_service = 12;
    actuatorPacket.header.destination = static_cast<uint8_t>(GeneralConfig::NetworkConfig::CHAD_SLAVE);

    actuatorPacket.arg = command.act2Deg;
    m_system.networkmanager.sendPacket(actuatorPacket);
}

bool Toaster::upperEndstopReached() {
    m_upperEndstopPressed = digitalRead(m_endstopUpper);
    return m_upperEndstopPressed;
}

bool Toaster::lowerEndstopReached() {
    m_lowerEndstopPressed = digitalRead(m_endstopLower);
    return m_lowerEndstopPressed;
}

void Toaster::commandTorque(const double torque) {
    const double angle = rocketTorqueToAngle(torque);

    const uint16_t commandAngle = static_cast<uint16_t>(angle * 10.0);

    const uint16_t commandAngleConstrained = std::clamp(
        commandAngle,
        static_cast<uint16_t>(GeneralConfig::act0ZeroAngle - GeneralConfig::actMaxAngle),
        static_cast<uint16_t>(GeneralConfig::act0ZeroAngle + GeneralConfig::actMaxAngle));

    actuatorPayload.act0Deg = GeneralConfig::act0ZeroAngle + commandAngleConstrained;
    actuatorPayload.act1Deg = GeneralConfig::act1ZeroAngle + commandAngleConstrained;
    actuatorPayload.act2Deg = GeneralConfig::act2ZeroAngle + commandAngleConstrained;

    actuatorCommand(actuatorPayload);
}

void Toaster::updatePIDLog(const PID::Log& pidLog) {
    m_pidLog = pidLog;
}

void Toaster::sendNetwork(const NRCPacket::NRC_COMMAND_ID& cmd) {
    SimpleCommandPacket armCommand(static_cast<uint32_t>(cmd), 0);
    armCommand.header.source = m_system.networkmanager.getAddress();
    armCommand.header.source_service = static_cast<uint8_t>(Services::ID::TOASTER);

    armCommand.header.destination_service = 10;
    armCommand.header.destination = static_cast<uint8_t>(GeneralConfig::NetworkConfig::CHAD_MASTER);
    m_system.networkmanager.sendPacket(armCommand);

    armCommand.header.destination_service = 11;
    armCommand.header.destination = static_cast<uint8_t>(GeneralConfig::NetworkConfig::CHAD_MASTER);
    m_system.networkmanager.sendPacket(armCommand);

    armCommand.header.destination_service = 10;
    armCommand.header.destination = static_cast<uint8_t>(GeneralConfig::NetworkConfig::CHAD_SLAVE);
    m_system.networkmanager.sendPacket(armCommand);

    m_toasterMachine.changeState(std::make_unique<Armed>(m_system, m_toasterStatus));
}

void Toaster::arm_base(int32_t arg) {
    switch (arg) {
    case 0: {
        // Transition to armed state
        if (m_toasterMachine.getCurrentStateID() == TOASTER_FLAGS::STATE_DEFAULT) {
            sendNetwork(NRCPacket::NRC_COMMAND_ID::ARM);
            m_toasterMachine.changeState(std::make_unique<Armed>(m_system, m_toasterStatus));
        } else {
            RicCoreLogging::log<RicCoreLoggingConfig::LOGGERS::SYS>("Not in default, cannot arm");
        }
        break;
    }
    case 1: {
        // Zero the stepper motor against the endpoint before arming
        if (m_toasterMachine.getCurrentStateID() == TOASTER_FLAGS::STATE_DEFAULT) {
            sendNetwork(NRCPacket::NRC_COMMAND_ID::ARM);
            m_toasterMachine.changeState(std::make_unique<Zero>(m_system, m_toasterStatus));
        } else {
            RicCoreLogging::log<RicCoreLoggingConfig::LOGGERS::SYS>("Not in default, cannot zero");
        }

        break;
    }
    case 2: {
        if (m_toasterMachine.getCurrentStateID() == TOASTER_FLAGS::STATE_ARMED) {
            m_toasterMachine.changeState(std::make_unique<Command>(m_system, m_toasterStatus));
        } else {
            RicCoreLogging::log<RicCoreLoggingConfig::LOGGERS::SYS>("Cannot move to command state, Toaster not armed");
        }
        break;
    }
    }
}

void Toaster::disarm_base() {
    sendNetwork(NRCPacket::NRC_COMMAND_ID::DISARM);
    m_toasterMachine.changeState(std::make_unique<ToasterDefault>(m_system, m_toasterStatus));
}

void Toaster::execute_base(int32_t arg) {
    switch (arg) {
    }
}

void Toaster::execute_impl(packetptr_t packetptr) {
    SimpleCommandPacket execute_command(*packetptr);

    switch (execute_command.arg) {
    }

    execute_base(execute_command.arg);
}

// Extra states for use during system debugging
void Toaster::extendedCommandHandler_impl(const NRCPacket::NRC_COMMAND_ID commandID, packetptr_t packetptr)
{
    SimpleCommandPacket command_packet(*packetptr);
    switch (static_cast<uint8_t>(commandID)) {
    // Telemetry command
    case 8: {
        m_toasterTelem.header.destination = command_packet.header.source;
        m_toasterTelem.header.destination_service = command_packet.header.source_service;
        m_toasterTelem.header.source = command_packet.header.destination;
        m_toasterTelem.header.source_service = command_packet.header.destination_service;
        m_toasterTelem.header.uid = command_packet.header.uid;

        m_toasterTelem.systemTime = millis();
        m_toasterTelem.systemStatus = m_toasterStatus.getStatus();
        m_toasterTelem.upperEndtopPressed = m_upperEndstopPressed;
        m_toasterTelem.lowerEndtopPressed = m_lowerEndstopPressed;
        m_toasterTelem.act0Position = actuatorPayload.act0Deg;
        m_toasterTelem.act1Position = actuatorPayload.act1Deg;
        m_toasterTelem.act2Position = actuatorPayload.act2Deg;
        m_toasterTelem.stepperEnabled = m_stepperEnabled;

        m_system.networkmanager.sendPacket(m_toasterTelem);
        break;
    }
    // PID Telemetry Command
    case 9: {
        m_pidTelem.header.destination = command_packet.header.source;
        m_pidTelem.header.destination_service = command_packet.header.source_service;
        m_pidTelem.header.source = command_packet.header.destination;
        m_pidTelem.header.source_service = command_packet.header.destination_service;
        m_pidTelem.header.uid = command_packet.header.uid;

        m_pidTelem.measurement = m_pidLog.measurement;
        m_pidTelem.target = m_pidLog.target;
        m_pidTelem.error = m_pidLog.error;
        m_pidTelem.kp = m_pidLog.kp;
        m_pidTelem.ki = m_pidLog.ki;
        m_pidTelem.kd = m_pidLog.kd;
        m_pidTelem.time = m_pidLog.time;
        m_pidTelem.control = m_pidLog.control;

        m_system.networkmanager.sendPacket(m_pidTelem);
        break;
    }
    default: {
        NRCRemoteActuatorBase::extendedCommandHandler_impl(commandID, std::move(packetptr));
        break;
    }
    }
}

double Toaster::rocketTorqueToAngle(const double torque) {
    return torque;
}
