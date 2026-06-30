import importlib
import sys
import types
import unittest
from pathlib import Path


class FakeLogger:
    def info(self, *_args, **_kwargs):
        pass

    def warn(self, *_args, **_kwargs):
        pass


class FakeParameter:
    def __init__(self, value):
        self.value = value


class FakeTime:
    nanoseconds = 0


class FakeClock:
    def now(self):
        return FakeTime()


class FakePublisher:
    def __init__(self):
        self.messages = []

    def publish(self, msg):
        self.messages.append(msg)


class FakeNode:
    def __init__(self, _name):
        self._params = {}

    def declare_parameter(self, name, value):
        self._params[name] = value

    def get_parameter(self, name):
        return FakeParameter(self._params[name])

    def create_publisher(self, *_args, **_kwargs):
        return FakePublisher()

    def create_subscription(self, *_args, **_kwargs):
        return object()

    def create_timer(self, *_args, **_kwargs):
        return object()

    def get_logger(self):
        return FakeLogger()

    def get_clock(self):
        return FakeClock()


class Float64MultiArray:
    def __init__(self):
        self.data = []


class Bool:
    def __init__(self):
        self.data = False


def import_controller_with_fakes():
    package_root = Path(__file__).resolve().parents[1]
    if str(package_root) not in sys.path:
        sys.path.insert(0, str(package_root))

    rclpy = types.ModuleType("rclpy")
    rclpy.init = lambda args=None: None
    rclpy.spin = lambda node: None
    rclpy.ok = lambda: True
    rclpy.shutdown = lambda: None

    rclpy_node = types.ModuleType("rclpy.node")
    rclpy_node.Node = FakeNode

    std_msgs = types.ModuleType("std_msgs")
    std_msgs_msg = types.ModuleType("std_msgs.msg")
    std_msgs_msg.Bool = Bool
    std_msgs_msg.Float64MultiArray = Float64MultiArray

    sys.modules["rclpy"] = rclpy
    sys.modules["rclpy.node"] = rclpy_node
    sys.modules["std_msgs"] = std_msgs
    sys.modules["std_msgs.msg"] = std_msgs_msg
    sys.modules.pop("amd_uw_ros2.manipulator_controller", None)
    return importlib.import_module("amd_uw_ros2.manipulator_controller")


def pickup_request(target_index=0):
    msg = Float64MultiArray()
    msg.data = [float(target_index), 1.0, 2.0]
    return msg


def arm_status(command_seq, state, target_index, success, error_code):
    msg = Float64MultiArray()
    msg.data = [float(command_seq), float(state), float(target_index), float(success), float(error_code)]
    return msg


class ManipulatorControllerTest(unittest.TestCase):
    def test_pickup_request_is_deduplicated_by_active_target(self):
        module = import_controller_with_fakes()
        node = module.ManipulatorController()

        node.on_pickup_request(pickup_request(0))
        node.on_pickup_request(pickup_request(0))

        self.assertEqual(len(node.arm_cmd_pub.messages), 1)
        command_seq = node.arm_cmd_pub.messages[0].data[0]
        self.assertGreater(command_seq, 0.0)
        self.assertEqual(node.arm_cmd_pub.messages[0].data[1:], [0.0, 1.0, 2.0])

    def test_done_status_publishes_target_done(self):
        module = import_controller_with_fakes()
        node = module.ManipulatorController()

        node.on_pickup_request(pickup_request(0))
        command_seq = node.arm_cmd_pub.messages[0].data[0]
        node.on_arm_status(arm_status(command_seq, module.ARM_STATE_DONE, 0, 1, 0))

        self.assertEqual(len(node.target_done_pub.messages), 1)
        self.assertIs(node.target_done_pub.messages[0].data, True)
        self.assertIsNone(node.active)

    def test_active_command_republishes_until_status_is_seen(self):
        module = import_controller_with_fakes()
        node = module.ManipulatorController()
        now_s = 0.0
        node.now_seconds = lambda: now_s

        node.on_pickup_request(pickup_request(0))
        command_seq = node.arm_cmd_pub.messages[0].data[0]
        now_s = 1.1
        node.on_timer()

        self.assertEqual(len(node.arm_cmd_pub.messages), 2)
        self.assertEqual(node.arm_cmd_pub.messages[1].data, [command_seq, 0.0, 1.0, 2.0])

        node.on_arm_status(arm_status(command_seq, module.ARM_STATE_BUSY, 0, 0, 0))
        now_s = 2.2
        node.on_timer()

        self.assertEqual(len(node.arm_cmd_pub.messages), 2)

    def test_failed_status_is_skipped_by_default(self):
        module = import_controller_with_fakes()
        node = module.ManipulatorController()

        node.on_pickup_request(pickup_request(0))
        command_seq = node.arm_cmd_pub.messages[0].data[0]
        node.on_arm_status(arm_status(command_seq, module.ARM_STATE_FAILED, 0, 0, 2))

        self.assertEqual(len(node.target_done_pub.messages), 1)
        self.assertIs(node.target_done_pub.messages[0].data, True)
        self.assertIsNone(node.active)


if __name__ == "__main__":
    unittest.main()
