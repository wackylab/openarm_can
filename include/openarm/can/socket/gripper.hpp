// Copyright 2025 Enactic, Inc.
//
// Licensed under the Apache License, Version 2.0 (the "License");
// you may not use this file except in compliance with the License.
// You may obtain a copy of the License at
//
//     http://www.apache.org/licenses/LICENSE-2.0
//
// Unless required by applicable law or agreed to in writing, software
// distributed under the License is distributed on an "AS IS" BASIS,
// WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.
// See the License for the specific language governing permissions and
// limitations under the License.

#pragma once

#include <mutex>

#include "gripper_component.hpp"
#include "../../canbus/can_socket.hpp"

namespace openarm::can::socket {

// High-level gripper wrapper that maintains position while respecting a force limit.
class Gripper {
public:
    struct State {
        double position;      // measured gripper position
        double velocity;      // measured gripper velocity
        double force;         // measured force/torque
        bool force_holding;   // true if in force-limit mode
        double max_effort;    // currently set maximum allowable effort
    };

    explicit Gripper(canbus::CANSocket& can_socket);
    ~Gripper() = default;

    // Command position [0..1].
    void set_position(double position);
    // Set maximum allowable effort before switching to force hold.
    void set_force(double max_effort);
    // Current state snapshot.
    State get_state() const;
    // Configure position controller gains.
    void set_pid(double kp, double kd);

    // Convenience operations.
    void open();
    void close();

    inline auto get_motors() const {return component_->get_motors();}

    // Internal wiring
    void init_motor_device(damiao_motor::MotorType motor_type, uint32_t send_can_id,
                           uint32_t recv_can_id, bool use_fd);
    void update();

private:
    friend class OpenArm;

    enum class ControlMode { Position, Force };

    GripperComponent& component() { return *component_; }

    std::unique_ptr<GripperComponent> component_;
    ControlMode mode_{ControlMode::Position};

    mutable std::mutex state_mutex_;
    double desired_position_{0.0};
    double desired_max_effort_{10.0};
    double position_kp_{10.0};
    double position_kd_{1.0};

    double last_force_sign_{1.0};
    const double hysteresis_ratio_{0.9};
};

}  // namespace openarm::can::socket
