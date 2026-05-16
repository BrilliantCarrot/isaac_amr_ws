#!/usr/bin/env python3

from dataclasses import dataclass
from typing import List, Optional

import rclpy
from rclpy.duration import Duration
from rclpy.node import Node
from rclpy.parameter import Parameter
from sensor_msgs.msg import JointState
from std_srvs.srv import Trigger


@dataclass(frozen=True)
class MotionStep:
    name: str
    joint_names: List[str]
    positions: List[float]
    settle_duration_sec: float


class FrankaBasicMotionNode(Node):
    """Publish hard-coded Franka JointState commands for W6 Isaac Sim validation."""

    def __init__(self):
        super().__init__("franka_basic_motion_node")

        self.declare_parameter("joint_command_topic", "/franka/joint_command")
        self.declare_parameter("auto_start", True)
        self.declare_parameter("command_repeat_count", 5)
        self.declare_parameter("command_repeat_period_sec", 0.10)
        self.declare_parameter("settle_duration_sec", 2.00)
        self.declare_parameter("gripper_settle_duration_sec", 1.00)
        self.declare_parameter("arm_joint_names", Parameter.Type.STRING_ARRAY)
        self.declare_parameter("gripper_joint_names", Parameter.Type.STRING_ARRAY)
        self.declare_parameter("home_positions", Parameter.Type.DOUBLE_ARRAY)
        self.declare_parameter("pre_pick_positions", Parameter.Type.DOUBLE_ARRAY)
        self.declare_parameter("pick_positions", Parameter.Type.DOUBLE_ARRAY)
        self.declare_parameter("lift_positions", Parameter.Type.DOUBLE_ARRAY)
        self.declare_parameter("pre_place_positions", Parameter.Type.DOUBLE_ARRAY)
        self.declare_parameter("place_positions", Parameter.Type.DOUBLE_ARRAY)
        self.declare_parameter("gripper_open_positions", Parameter.Type.DOUBLE_ARRAY)
        self.declare_parameter("gripper_closed_positions", Parameter.Type.DOUBLE_ARRAY)

        self.command_topic = self.get_parameter("joint_command_topic").value
        self.arm_joint_names = self._string_list_param("arm_joint_names")
        self.gripper_joint_names = self._string_list_param("gripper_joint_names")
        self.repeat_count = int(self.get_parameter("command_repeat_count").value)
        self.repeat_period_sec = float(self.get_parameter("command_repeat_period_sec").value)
        self.arm_settle_sec = float(self.get_parameter("settle_duration_sec").value)
        self.gripper_settle_sec = float(
            self.get_parameter("gripper_settle_duration_sec").value
        )

        self.home_positions = self._float_list_param("home_positions")
        self.pre_pick_positions = self._float_list_param("pre_pick_positions")
        self.pick_positions = self._float_list_param("pick_positions")
        self.lift_positions = self._float_list_param("lift_positions")
        self.pre_place_positions = self._float_list_param("pre_place_positions")
        self.place_positions = self._float_list_param("place_positions")
        self.gripper_open_positions = self._float_list_param("gripper_open_positions")
        self.gripper_closed_positions = self._float_list_param("gripper_closed_positions")

        self._validate_config()

        self.command_pub = self.create_publisher(JointState, self.command_topic, 10)
        self.start_srv = self.create_service(Trigger, "~/start", self._on_start)
        self.home_srv = self.create_service(Trigger, "~/home", self._on_home)
        self.open_srv = self.create_service(Trigger, "~/open_gripper", self._on_open_gripper)
        self.close_srv = self.create_service(Trigger, "~/close_gripper", self._on_close_gripper)

        self.sequence: List[MotionStep] = []
        self.current_step_index = 0
        self.active_step: Optional[MotionStep] = None
        self.repeat_remaining = 0
        self.next_publish_time = self.get_clock().now()
        self.next_step_time = self.get_clock().now()
        self.running = False
        self.done_reported = False

        self.timer = self.create_timer(0.02, self._on_timer)

        self.get_logger().info(
            f"Franka basic motion node ready. Publishing JointState commands to {self.command_topic}"
        )

        if self.get_parameter("auto_start").value:
            self._start_sequence()

    def _string_list_param(self, name: str) -> List[str]:
        return list(self.get_parameter(name).get_parameter_value().string_array_value)

    def _float_list_param(self, name: str) -> List[float]:
        return list(self.get_parameter(name).get_parameter_value().double_array_value)

    def _validate_config(self):
        arm_poses = [
            ("home_positions", self.home_positions),
            ("pre_pick_positions", self.pre_pick_positions),
            ("pick_positions", self.pick_positions),
            ("lift_positions", self.lift_positions),
            ("pre_place_positions", self.pre_place_positions),
            ("place_positions", self.place_positions),
        ]
        for name, positions in arm_poses:
            if len(positions) != len(self.arm_joint_names):
                raise ValueError(
                    f"{name} has {len(positions)} values, but "
                    f"{len(self.arm_joint_names)} arm joints are configured"
                )

        gripper_poses = [
            ("gripper_open_positions", self.gripper_open_positions),
            ("gripper_closed_positions", self.gripper_closed_positions),
        ]
        for name, positions in gripper_poses:
            if len(positions) != len(self.gripper_joint_names):
                raise ValueError(
                    f"{name} has {len(positions)} values, but "
                    f"{len(self.gripper_joint_names)} gripper joints are configured"
                )

        if self.repeat_count < 1:
            raise ValueError("command_repeat_count must be >= 1")

    def _make_sequence(self) -> List[MotionStep]:
        return [
            self._arm_step("home", self.home_positions),
            self._arm_step("pre_pick", self.pre_pick_positions),
            self._arm_step("pick", self.pick_positions),
            self._gripper_step("gripper_close", self.gripper_closed_positions),
            self._arm_step("lift", self.lift_positions),
            self._arm_step("pre_place", self.pre_place_positions),
            self._arm_step("place", self.place_positions),
            self._gripper_step("gripper_open", self.gripper_open_positions),
            self._arm_step("home", self.home_positions),
        ]

    def _arm_step(self, name: str, positions: List[float]) -> MotionStep:
        return MotionStep(name, self.arm_joint_names, positions, self.arm_settle_sec)

    def _gripper_step(self, name: str, positions: List[float]) -> MotionStep:
        return MotionStep(
            name,
            self.gripper_joint_names,
            positions,
            self.gripper_settle_sec,
        )

    def _on_start(self, request, response):
        del request
        if self.running:
            response.success = False
            response.message = "sequence already running"
            return response

        self._start_sequence()
        response.success = True
        response.message = "sequence started"
        return response

    def _on_home(self, request, response):
        del request
        self._publish_command(self.arm_joint_names, self.home_positions)
        response.success = True
        response.message = "home command published"
        return response

    def _on_open_gripper(self, request, response):
        del request
        self._publish_command(self.gripper_joint_names, self.gripper_open_positions)
        response.success = True
        response.message = "open gripper command published"
        return response

    def _on_close_gripper(self, request, response):
        del request
        self._publish_command(self.gripper_joint_names, self.gripper_closed_positions)
        response.success = True
        response.message = "close gripper command published"
        return response

    def _start_sequence(self):
        self.sequence = self._make_sequence()
        self.current_step_index = 0
        self.running = True
        self.done_reported = False
        self._activate_current_step()

    def _activate_current_step(self):
        if self.current_step_index >= len(self.sequence):
            self.running = False
            self.active_step = None
            return

        self.active_step = self.sequence[self.current_step_index]
        self.repeat_remaining = self.repeat_count
        now = self.get_clock().now()
        self.next_publish_time = now
        self.next_step_time = now + Duration(seconds=self.active_step.settle_duration_sec)
        self.get_logger().info(
            f"step {self.current_step_index + 1}/{len(self.sequence)}: {self.active_step.name}"
        )

    def _on_timer(self):
        if not self.running:
            if not self.done_reported and self.sequence:
                self.get_logger().info("Franka basic motion sequence done")
                self.done_reported = True
            return

        now = self.get_clock().now()
        if self.active_step is None:
            self._activate_current_step()
            return

        if self.repeat_remaining > 0 and now >= self.next_publish_time:
            self._publish_command(self.active_step.joint_names, self.active_step.positions)
            self.repeat_remaining -= 1
            self.next_publish_time = now + Duration(seconds=self.repeat_period_sec)

        if now >= self.next_step_time:
            self.current_step_index += 1
            self._activate_current_step()

    def _publish_command(self, joint_names: List[str], positions: List[float]):
        msg = JointState()
        msg.header.stamp = self.get_clock().now().to_msg()
        msg.name = list(joint_names)
        msg.position = [float(value) for value in positions]
        self.command_pub.publish(msg)


def main(args=None):
    rclpy.init(args=args)
    node = FrankaBasicMotionNode()
    try:
        rclpy.spin(node)
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == "__main__":
    main()
