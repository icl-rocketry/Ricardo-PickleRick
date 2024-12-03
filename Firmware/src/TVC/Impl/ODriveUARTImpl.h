// MIT License

// Copyright (c) 2017 Oskar Weigl

// Permission is hereby granted, free of charge, to any person obtaining a copy
// of this software and associated documentation files (the "Software"), to deal
// in the Software without restriction, including without limitation the rights
// to use, copy, modify, merge, publish, distribute, sublicense, and/or sell
// copies of the Software, and to permit persons to whom the Software is
// furnished to do so, subject to the following conditions:

// The above copyright notice and this permission notice shall be included in all
// copies or substantial portions of the Software.

// THE SOFTWARE IS PROVIDED "AS IS", WITHOUT WARRANTY OF ANY KIND, EXPRESS OR
// IMPLIED, INCLUDING BUT NOT LIMITED TO THE WARRANTIES OF MERCHANTABILITY,
// FITNESS FOR A PARTICULAR PURPOSE AND NONINFRINGEMENT. IN NO EVENT SHALL THE
// AUTHORS OR COPYRIGHT HOLDERS BE LIABLE FOR ANY CLAIM, DAMAGES OR OTHER
// LIABILITY, WHETHER IN AN ACTION OF CONTRACT, TORT OR OTHERWISE, ARISING FROM,
// OUT OF OR IN CONNECTION WITH THE SOFTWARE OR THE USE OR OTHER DEALINGS IN THE
// SOFTWARE.

#pragma once

#include "Arduino.h"
#include "ODriveUARTEnums.h"

class ODriveArduino {
public:
    enum class ODriveSysCommand {
        REBOOT,
        SAVE_CONF,
        ERASE_CONF,
        CLEAR_ERR
    };

    ODriveArduino(Stream& serial);

    // Commands
    void SetPosition(int motor_number, float position);
    void SetPosition(int motor_number, float position, float velocity_feedforward);
    void SetPosition(int motor_number, float position, float velocity_feedforward, float current_feedforward);
    void SetVelocity(int motor_number, float velocity);
    void SetVelocity(int motor_number, float velocity, float current_feedforward);
    void SetCurrent(int motor_number, float current);
    void TrapezoidalMove(int motor_number, float position);
    // Getters
    float GetVelocity(int motor_number);
    float GetPosition(int motor_number);
    // General params
    float readFloat();
    int32_t readInt();

    // State helper
    bool run_state(int axis, int requested_state, bool wait_for_idle, float timeout = 10.0f);

    void WriteConfig(const std::string& config, const float value);
    void ReadConfig(const std::string& config);
    void SendSystemCommand(ODriveSysCommand command);
private:
    String readString();

    Stream& serial_;
};