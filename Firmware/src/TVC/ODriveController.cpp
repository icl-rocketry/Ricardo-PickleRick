#include "ODriveController.h"

#include <ArduinoJson.h>
#include <librrc/Helpers/jsonconfighelper.h>
#include <driver/twai.h>

//! @brief CAN Header IDs for received messages.
enum RXHeaderID {
    GET_VERSION = 0x000,
    HEARTBEAT = 0x001,
    GET_ERROR = 0x003,
    TXSDO = 0x005,
    ADDRESS = 0x006,
    GET_ENCODER_ESTIMATES = 0x009,
    GET_IQ = 0x014,
    GET_TEMPERATURE = 0x015,
    GET_BUS_VOLTAGE_CURRENT = 0x017,
    GET_POWERS = 0x01d,
    GET_TORQUES = 0x01c,
};

//! @brief CAN Header IDs for transmitting messages.
enum TXHeaderID {
    ESTOP = 0x002,
    RXSDO = 0x004,
    ADDRESS = 0x006,
    SET_AXIS_STATE = 0x007,
    SET_CONTROLLER_MODE = 0x00b,
    SET_INPUT_POS = 0x00c,
    SET_INPUT_VEL = 0x00d,
    SET_INPUT_TORQUE = 0x00e,
    SET_LIMITS = 0x00f,
    SET_TRAJ_VEL_LIMIT = 0x011,
    SET_TRAJ_ACCEL_LIMITS = 0x012,
    SET_TRAJ_INERTIA     = 0x013,
    REBOOT = 0x016,
    CLEAR_ERRORS = 0x018,
    SET_ABSOLUTE_POSITION = 0x019,
    SET_POS_GAIN = 0x01a,
    SET_VEL_GAINS = 0x01b,
    ENTER_DFU_MODE = 0x01f,
};

//! @brief Baud rate of UART connection for the ODrive
const unsigned int UART_BAUD = 115200;

ODriveController::ODriveController() {}

ODriveController ODriveController::fromConfig(JsonObjectConst config) {
    std::string connectionType = LIBRRC::JsonConfigHelper::getIfContains<std::string>(config, "connection_type");

    ODriveController controller;

    if (connectionType == "CAN") {
        controller.connection = Connection::CAN;

        // CAN controller not supported for now since there is no current easy
        // way to receive CAN messages from the ODrive since it uses a 
        // different format.
        // This is TODO
        throw "[ODriveController] CAN controller not supported";

    } else if (connectionType == "UART") {
        controller.connection = Connection::UART;

        int uartTxPin = LIBRRC::JsonConfigHelper::getIfContains<int>(config, "uart_tx_pin");
        int uartRxPin = LIBRRC::JsonConfigHelper::getIfContains<int>(config, "uart_rx_pin");

        controller.uartTxPin = uartTxPin;
        controller.uartRxPin = uartRxPin;


    } else {
        throw "[ODriveController] Invalid connection config type";
    }

    float turnRange = LIBRRC::JsonConfigHelper::getIfContains<float>(config, "turn_range");
    controller.turnRange = turnRange;

    float currentTurns = LIBRRC::JsonConfigHelper::getIfContains<float>(config, "turn_initial");
    controller.currentTurns = std::min(currentTurns, turnRange);

    return controller;
}

bool ODriveController::arm() {
    return false;
}

bool ODriveController::status() {
    return false;
}

void ODriveController::printDebug() {
}

void ODriveController::position(float xAxis, float yAxis) const {
    assert(std::abs(xAxis) <= 1.0 && std::abs(yAxis) <= 1.0);

    float xAxisTurns = xAxis * turnRange - currentTurns;
    float yAxisTurns = yAxis * turnRange - currentTurns;
}

bool ODriveController::restart(RestartType type) {
    return false;
}

ODriveController::operator bool() const {
    return operational;
};