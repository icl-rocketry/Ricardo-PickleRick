#pragma once

#include <cinttypes>
#include <ArduinoJson.h>
#include <librrc/Helpers/jsonconfighelper.h>

#include "Impl/ODriveUARTImpl.h"

class ODriveController {
public:
    /**
     * @brief How the board is connected to the ODrive.
     */
    enum Connection {
        UART,
        CAN
    };

    /**
     * @brief Privately construct a new ODriveController object using a serial Stream.
     */
    ODriveController(Stream& serial);

    /** @brief Factory method for constructing the ODrive controller from a config setup. */    
    static ODriveController fromConfig(JsonObjectConst config);

    /**
     * @brief Arms the controller.
     * 
     * @return true Success
     * @return false Failure
     */
    bool arm();

    /**
     * @brief Query the status of the controller.
     * 
     * @return true All good!
     * @return false Bad
     */
    bool status();

    /**
     * @brief Queries the ODrive for some debug information and prints.
     */
    void printDebug();

    /**
     * @brief Send a position command to the ODrive controller.
     * 
     * Commands should be in the range [0, 1], 0 representing the 
     * minimal position, and 1 the maximal.
     * 
     * @param xAxis Position command for the motor on the x-axis (Motor 0).
     * @param yAxis Position command for the motor on the y-axis (Motor 1).
     */
    void position(float xAxis, float yAxis);

    using SysCommand = ODriveArduino::ODriveSysCommand;

    /**
     * @brief Blocking call to restart the connected ODrive.
     * 
     * The call is blocking as it awaits a valid response from the ODrive.
     * 
     * @param type The type of restart call.
     */
    void command(SysCommand type);

    /**
     * @brief Query the status of the ODrive, returns wether or not the ODrive sent a 
     * valid heartbeat on the last construction / restart.
     * 
     * Use status() to query the board directly.
     * 
     * @return true 
     * @return false 
     */
    explicit operator bool();
private:

    //! @brief The status from the ODrive from the last status check
    bool operational = false;

    //! @brief How the controller is connected to this board.
    Connection connection;
    
    //! @brief Can address of ODrive
    int canTxAddr = -1;

    //! @brief Can address of this Board
    int canRxAddr = -1;

    //! @brief UART transmit pin
    int uartTxPin = -1;

    //! @brief UART receive pin
    int uartRxPin = -1;

    //! @brief Maximal number of turns of the motor between minimal and maximal position.
    float turnRange = 0;

    //! @brief Current number of turns from minimum.
    float currentTurns = 0;

    ODriveArduino uartDriver;
};