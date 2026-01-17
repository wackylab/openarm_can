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
    arm.init_gripper_motor(oa.MotorType.DM4310, 0x8, 0x18, oa.ControlMode.POS_FORCE)

    arm.set_callback_mode_all(oa.CallbackMode.PARAM)
    arm.enable_all()
    arm.recv_all()

    arm.set_callback_mode_all(oa.CallbackMode.STATE)
    gripper = arm.get_gripper()

    gripper.set_limit(6.0, 0.4)  # speed_rad_s, torque_pu
    gripper.open()
    time.sleep(0.4)

    gripper_motor = gripper.get_motors()[0]
    initial_kp_asr = gripper_motor.get_param(oa.MotorVariable.KP_ASR.value)
    initial_ki_asr = gripper_motor.get_param(oa.MotorVariable.KI_ASR.value)
    print(f"\n=== INITIAL KP/KI VALUES ===")
    print(f"KP_ASR (speed loop Kp): {initial_kp_asr}")
    print(f"KI_ASR (speed loop Ki): {initial_ki_asr}")
    print(f"===========================\n")

    sequence = [
        (0.25, 3.0, 0.2),
        (0.25, 3.0, 0.6),
        (0.6, 2.0, 0.3),
        (0.6, 2.0, 0.8),
        (0.4, 6.0, 0.25),
        (0.4, 6.0, 0.7),
    ]

    print("=== FIRST HALF: Using DEFAULT KP/KI ===")
    for position, speed, torque in sequence[:3]:
        print(f"set_position({position}) speed={speed} torque={torque}")
        gripper.set_position(position, speed_rad_s=speed, torque_pu=torque)
        for _ in range(6):
            arm.refresh_all()
            arm.recv_all(500)
            for motor in gripper.get_motors():
                print("gripper position:", motor.get_position())
            time.sleep(0.05)

    new_kp_asr = 80.0
    new_ki_asr = 2.0
    print(f"\n=== WRITING NEW KP/KI VALUES ===")
    print(f"Setting KP_ASR to: {new_kp_asr}")
    print(f"Setting KI_ASR to: {new_ki_asr}")
    gripper.set_param_one(0, oa.MotorVariable.KP_ASR.value, new_kp_asr)
    gripper.set_param_one(0, oa.MotorVariable.KI_ASR.value, new_ki_asr)
    time.sleep(0.1)

    readback_kp_asr = gripper_motor.get_param(oa.MotorVariable.KP_ASR.value)
    readback_ki_asr = gripper_motor.get_param(oa.MotorVariable.KI_ASR.value)
    print(f"KP_ASR after write: {readback_kp_asr}")
    print(f"KI_ASR after write: {readback_ki_asr}")
    print(f"===============================\n")

    print("=== SECOND HALF: Using NEW KP/KI (faster response) ===")
    for position, speed, torque in sequence[3:]:
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
