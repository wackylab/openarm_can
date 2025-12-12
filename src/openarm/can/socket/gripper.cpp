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

#include <algorithm>
#include <cmath>

#include <openarm/can/socket/gripper.hpp>

namespace openarm::can::socket {

Gripper::Gripper(canbus::CANSocket& can_socket)
    : component_(std::make_unique<GripperComponent>(can_socket)) {}

void Gripper::init_motor_device(damiao_motor::MotorType motor_type, uint32_t send_can_id,
                                uint32_t recv_can_id, bool use_fd) {
    component_->init_motor_device(motor_type, send_can_id, recv_can_id, use_fd);
}

void Gripper::set_position(double position) {
    std::lock_guard<std::mutex> lock(state_mutex_);
    desired_position_ = std::clamp(position, 0.0, 1.0);
    mode_ = ControlMode::Position;
}

void Gripper::set_force(double max_effort) {
    std::lock_guard<std::mutex> lock(state_mutex_);
    desired_max_effort_ = std::abs(max_effort);
}

Gripper::State Gripper::get_state() const {
    Gripper::State state{};
    state.position = std::clamp(component_->get_measured_position(), 0.0, 1.0);
    state.velocity = component_->get_motor()->get_velocity();
    state.force = component_->get_measured_force();
    {
        std::lock_guard<std::mutex> lock(state_mutex_);
        state.force_holding = mode_ == ControlMode::Force;
        state.max_effort = desired_max_effort_;
    }
    return state;
}

void Gripper::open() { set_position(1.0); }

void Gripper::close() { set_position(0.0); }

void Gripper::set_pid(double kp, double kd) {
    std::lock_guard<std::mutex> lock(state_mutex_);
    position_kp_ = kp;
    position_kd_ = kd;
}

void Gripper::update() {
    double cmd_position = 0.0;
    double max_effort = 0.0;
    ControlMode mode = ControlMode::Position;
    {
        std::lock_guard<std::mutex> lock(state_mutex_);
        cmd_position = desired_position_;
        max_effort = desired_max_effort_;
        mode = mode_;
    }

    const double measured_force = component_->get_measured_force();
    const double force_magnitude = std::abs(measured_force);

    // Force-limit logic with hysteresis.
    const bool force_limit_enabled = max_effort > 0.0;
    if (force_limit_enabled && force_magnitude > max_effort && mode == ControlMode::Position) {
        mode = ControlMode::Force;
    } else if (mode == ControlMode::Force) {
        const double release_threshold = max_effort * hysteresis_ratio_;
        if (!force_limit_enabled || force_magnitude < release_threshold) {
            mode = ControlMode::Position;
        }
    }

    if (mode == ControlMode::Force && force_limit_enabled) {
        const double sign =
            (measured_force == 0.0) ? last_force_sign_ : (measured_force > 0.0 ? 1.0 : -1.0);
        last_force_sign_ = sign;
        component_->apply_force(sign * max_effort);
    } else {
        component_->set_position(cmd_position, position_kp_, position_kd_);
        if (measured_force != 0.0) {
            last_force_sign_ = measured_force > 0.0 ? 1.0 : -1.0;
        }
    }

    std::lock_guard<std::mutex> lock(state_mutex_);
    mode_ = mode;
}

}  // namespace openarm::can::socket
