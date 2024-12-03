#include "ODriveController.h"

#include <ArduinoJson.h>
#include <librrc/Helpers/jsonconfighelper.h>
#include <driver/twai.h>

//! @brief Baud rate of UART connection for the ODrive
const unsigned int UART_BAUD = 115200;

const int X_AXIS = 0;
const int Y_AXIS = 1;

ODriveController::ODriveController(Stream& serial): uartDriver(serial) {}


// ODriveController ODriveController::fromConfig(JsonObjectConst config) {
//     std::string connectionType = LIBRRC::JsonConfigHelper::getIfContains<std::string>(config, "connection_type");

//     ODriveController controller;

//     if (connectionType == "CAN") {
//         controller.connection = Connection::CAN;

//         // CAN controller not supported for now since there is no current easy
//         // way to receive CAN messages from the ODrive since it uses a 
//         // different format.
//         // This is TODO
//         throw "[ODriveController] CAN controller not supported";

//     } else if (connectionType == "UART") {
//         controller.connection = Connection::UART;

//         int uartTxPin = LIBRRC::JsonConfigHelper::getIfContains<int>(config, "uart_tx_pin");
//         int uartRxPin = LIBRRC::JsonConfigHelper::getIfContains<int>(config, "uart_rx_pin");

//         controller.uartTxPin = uartTxPin;
//         controller.uartRxPin = uartRxPin;


//     } else {
//         throw "[ODriveController] Invalid connection config type";
//     }

//     float turnRange = LIBRRC::JsonConfigHelper::getIfContains<float>(config, "turn_range");
//     controller.turnRange = turnRange;

//     float currentTurns = LIBRRC::JsonConfigHelper::getIfContains<float>(config, "turn_initial");
//     controller.currentTurns = std::min(currentTurns, turnRange);

//     return controller;
// }

bool ODriveController::arm() {
    return false;
}

bool ODriveController::status() {
    return false;
}

void ODriveController::printDebug() {
}

void ODriveController::position(float xAxis, float yAxis) {
    assert(std::abs(xAxis) <= 1.0 && std::abs(yAxis) <= 1.0);

    const float xAxisTurns = xAxis * turnRange - currentTurns;
    const float yAxisTurns = yAxis * turnRange - currentTurns;

    uartDriver.SetPosition(X_AXIS, xAxisTurns);
    uartDriver.SetPosition(Y_AXIS, yAxisTurns);
}

void ODriveController::command(SysCommand command) {
    uartDriver.SendSystemCommand(command);
}

ODriveController::operator bool() {
    return status();
};