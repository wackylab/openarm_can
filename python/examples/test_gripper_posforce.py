# Copyright 2025 Enactic, Inc.
#
# Licensed under the Apache License, Version 2.0 (the "License");
# you may not use this file except in compliance with the License.
# You may obtain a copy of the License at
#
#     http://www.apache.org/licenses/LICENSE-2.0
#
# Unless required by applicable law or agreed to in writing, software
# distributed under the License is distributed on an "AS IS" BASIS,
# WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.
# See the License for the specific language governing permissions and
# limitations under the License.

import time

import openarm_can as oa


def main() -> None:
    arm = oa.OpenArm("can0", True)
    arm.init_gripper_motor(oa.MotorType.DM4310, 0x8,
                           0x18, oa.ControlMode.POS_FORCE)

    arm.enable_all()
    arm.recv_all()

    gripper = arm.get_gripper()

    gripper.set_limit(6.0, 0.4)  # speed_rad_s, torque_pu
    gripper.open()
    time.sleep(0.4)

    # Query KP/KI parameters and receive response
    # NOTE: always clear previous messages before switching to PARAM callback mode
    arm.recv_all(500)
    arm.set_callback_mode_all(oa.CallbackMode.PARAM)
    print("Querying KP_ASR (speed loop Kp)...")
    print("Querying KI_ASR (speed loop Ki)...")
    print("Querying KP_APR (position loop Kp)...")
    print("Querying KI_APR (position loop Ki)...")
    gripper.query_param_one(0, oa.MotorVariable.KP_ASR.value)
    gripper.query_param_one(0, oa.MotorVariable.KI_ASR.value)
    gripper.query_param_one(0, oa.MotorVariable.KP_APR.value)
    gripper.query_param_one(0, oa.MotorVariable.KI_APR.value)
    arm.recv_all(500)
    arm.set_callback_mode_all(oa.CallbackMode.STATE)

    # Read cached values after querying
    gripper_motor = gripper.get_motors()[0]
    initial_kp_asr = gripper_motor.get_param(oa.MotorVariable.KP_ASR.value)
    initial_ki_asr = gripper_motor.get_param(oa.MotorVariable.KI_ASR.value)
    initial_kp_apr = gripper_motor.get_param(oa.MotorVariable.KP_APR.value)
    initial_ki_apr = gripper_motor.get_param(oa.MotorVariable.KI_APR.value)
    print(f"\n=== INITIAL KP/KI VALUES ===")
    print(f"KP_ASR (speed loop Kp): {initial_kp_asr}")
    print(f"KI_ASR (speed loop Ki): {initial_ki_asr}")
    print(f"KP_APR (position loop Kp): {initial_kp_apr}")
    print(f"KI_APR (position loop Ki): {initial_ki_apr}")
    print(f"===========================\n")

    print("Setting KP_APR to 200.0...")
    gripper.set_param_one(0, oa.MotorVariable.KP_APR.value, 200.0)
    time.sleep(0.1)

    arm.set_callback_mode_all(oa.CallbackMode.PARAM)
    gripper.query_param_one(0, oa.MotorVariable.KP_APR.value)
    arm.recv_all(500)
    arm.set_callback_mode_all(oa.CallbackMode.STATE)
    new_kp_apr = gripper.get_motors()[0].get_param(
        oa.MotorVariable.KP_APR.value)
    print(f"New KP_APR value: {new_kp_apr}\n")

    sequence_part1 = [
        (0.25, 3.0, 0.2),
        (0.25, 3.0, 0.6),
    ]

    print("\n--- Phase 1: KP_APR = 200.0 (fast response) ---")
    for position, speed, torque in sequence_part1:
        print(f"set_position({position}) speed={speed} torque={torque}")
        gripper.set_position(position, speed_rad_s=speed, torque_pu=torque)
        for _ in range(6):
            arm.refresh_all()
            arm.recv_all(500)
            for motor in gripper.get_motors():
                print("gripper position:", motor.get_position())
            time.sleep(0.05)

    print("\nChanging KP_APR to 10.0 (slow response)...")
    gripper.set_param_one(0, oa.MotorVariable.KP_APR.value, 10.0)
    time.sleep(0.1)
    arm.set_callback_mode_all(oa.CallbackMode.PARAM)
    gripper.query_param_one(0, oa.MotorVariable.KP_APR.value)
    arm.recv_all(500)
    arm.set_callback_mode_all(oa.CallbackMode.STATE)
    new_kp_apr = gripper.get_motors()[0].get_param(
        oa.MotorVariable.KP_APR.value)
    print(f"New KP_APR value: {new_kp_apr}")

    sequence_part2 = [
        (0.6, 2.0, 0.3),
        (0.6, 2.0, 0.8),
        (0.4, 6.0, 0.25),
        (0.4, 6.0, 0.7),
    ]

    print("\n--- Phase 2: KP_APR = 10.0 (slow response) ---")
    for position, speed, torque in sequence_part2:
        print(f"set_position({position}) speed={speed} torque={torque}")
        gripper.set_position(position, speed_rad_s=speed, torque_pu=torque)
        for _ in range(6):
            arm.refresh_all()
            arm.recv_all(500)
            for motor in gripper.get_motors():
                print("gripper position:", motor.get_position())
            time.sleep(0.05)

    gripper.grasp(0.1)  # torque_pu, speed_rad_s (optional)
    time.sleep(0.5)

    gripper.close()
    time.sleep(0.4)

    arm.disable_all()
    arm.recv_all()


if __name__ == "__main__":
    main()
