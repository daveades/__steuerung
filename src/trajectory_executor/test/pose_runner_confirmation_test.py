import copy
import math
import os
from pathlib import Path
import subprocess
import sys
import tempfile
import threading
import time
import unittest

import rclpy
from ament_index_python.packages import get_package_share_directory
from rclpy.executors import SingleThreadedExecutor
from rclpy.node import Node
from rclpy.qos import DurabilityPolicy, QoSProfile
from rcl_interfaces.msg import Log
from sensor_msgs.msg import JointState
from std_msgs.msg import Bool, String


os.environ['ROS_DOMAIN_ID'] = str(180 + os.getpid() % 40)
os.environ['ROS_LOCALHOST_ONLY'] = '1'
os.environ['RCUTILS_COLORIZED_OUTPUT'] = '0'


runner_path = str(Path(sys.argv.pop(1)).resolve())
rosout_qos = QoSProfile(depth=1000, durability=DurabilityPolicy.TRANSIENT_LOCAL)


class DelayedArm(Node):

    def __init__(self):
        super().__init__('delayed_test_arm')
        self.names = [f'arm_joint{i}' for i in range(1, 6)] + ['grip_joint']
        self.feedback_names = list(self.names)
        self.positions = [0.0] * 6
        self.delay_seconds = 0.35
        self.confirmation_mode = 'correct'
        self.move = True
        self.freeze_feedback = False
        self.feedback_stamp = None
        self.pending = []
        self.received_times = []
        self.sent_times = []
        self.rejected_times = []
        self.sent_commands = []
        self.position_offsets = {}
        self.offset_joint_name = 'arm_joint1'
        self.unconfirmed_commands = set()
        self.next_allowed_time = 0.0
        self.continuous_motion = False
        self.interpolate = False
        self.movement_start = None
        self.movement_positions = list(self.positions)
        self.movement_targets = list(self.positions)
        self.command_measurements = []
        self.feedback_publisher = self.create_publisher(JointState, 'joint_states', 10)
        self.sent_publisher = self.create_publisher(JointState, 'joint_command_sent', 10)
        self.command_subscription = self.create_subscription(
            JointState, 'joint_commands', self.receive_command, 1)
        self.timer = self.create_timer(0.02, self.tick)

    def receive_command(self, command):
        current_time = time.monotonic()
        self.received_times.append(current_time)
        self.command_measurements.append((list(self.positions), copy.deepcopy(command)))
        if current_time < self.next_allowed_time:
            self.rejected_times.append(current_time)
            return
        if self.freeze_feedback:
            self.apply_command(command)
            self.feedback_stamp = self.get_clock().now().to_msg()
        self.pending.append((current_time + self.delay_seconds, copy.deepcopy(command)))

    def apply_command(self, command):
        if self.interpolate:
            self.movement_positions = list(self.positions)
            self.movement_targets = list(self.positions)
            for name, position in zip(command.name, command.position):
                self.movement_targets[self.names.index(name)] = position
            self.movement_start = time.monotonic()
            return
        for name, position in zip(command.name, command.position):
            self.positions[self.names.index(name)] = position

    def enable_continuous_motion(self, interpolate=True):
        self.continuous_motion = True
        self.interpolate = interpolate
        self.mode_publisher = self.create_publisher(
            Bool, 'continuous_motion_enabled',
            QoSProfile(depth=1, durability=DurabilityPolicy.TRANSIENT_LOCAL))
        self.mode_publisher.publish(Bool(data=True))

    def tick(self):
        current_time = time.monotonic()
        if self.movement_start is not None:
            progress = min(current_time - self.movement_start, 1.0)
            self.positions = [
                start + (target - start) * progress
                for start, target in zip(self.movement_positions, self.movement_targets)]
        while self.pending and self.pending[0][0] <= current_time:
            _, command = self.pending.pop(0)
            self.sent_times.append(current_time)
            self.sent_commands.append(copy.deepcopy(command))
            command_number = len(self.sent_commands)
            self.next_allowed_time = current_time + (0.5 if self.continuous_motion else 2.0)
            if self.move:
                self.apply_command(command)
                self.positions[self.names.index(self.offset_joint_name)] += (
                    self.position_offsets.get(command_number, 0.0))
            if (self.confirmation_mode != 'missing' and
                    command_number not in self.unconfirmed_commands):
                if self.confirmation_mode == 'wrong_stamp':
                    command.header.stamp.sec += 1
                elif self.confirmation_mode == 'wrong_target':
                    command.position[0] += 0.001
                elif self.confirmation_mode == 'wrong_names':
                    if len(command.name) == 1:
                        command.name = ['arm_joint1']
                    else:
                        command.name.reverse()
                elif self.confirmation_mode == 'extra_joints':
                    command.name = list(self.names)
                    command.position = list(self.positions)
                self.sent_publisher.publish(command)
        feedback = JointState()
        feedback.header.stamp = self.feedback_stamp or self.get_clock().now().to_msg()
        feedback.name = self.feedback_names
        feedback.position = [
            self.positions[self.names.index(name)] for name in self.feedback_names]
        self.feedback_publisher.publish(feedback)


class PoseRunnerConfirmation(unittest.TestCase):

    def setUp(self):
        rclpy.init()
        self.arm = DelayedArm()
        self.executor = SingleThreadedExecutor()
        self.executor.add_node(self.arm)
        self.stop_event = threading.Event()
        self.thread = threading.Thread(target=self.spin, daemon=True)
        self.thread.start()
        self.addCleanup(self.shutdown)

    def spin(self):
        while not self.stop_event.is_set():
            self.executor.spin_once(timeout_sec=0.05)

    def shutdown(self):
        self.stop_event.set()
        self.thread.join(timeout=2)
        self.executor.shutdown()
        self.arm.destroy_node()
        rclpy.shutdown()

    def run_pose(self, target=0.04, execute=True):
        arguments = [
            runner_path, '--ros-args', '-p', 'pose_name:=confirmation_test',
            '-p', f'poses.confirmation_test:=[{target},0.0,0.0,0.0,0.0,0.0]',
            '-p', f'execute:={str(execute).lower()}',
            '-p', 'position_tolerance_degrees:=0.25',
        ]
        return self.run_runner(arguments)

    def run_gripper(self, target=-0.02, execute=True, extra_parameters=()):
        arguments = [runner_path, '--ros-args', '-p', f'gripper_target:={target}',
                     '-p', 'position_tolerance_degrees:=0.25']
        if execute is not None:
            arguments += ['-p', f'execute:={str(execute).lower()}']
        for parameter in extra_parameters:
            arguments += ['-p', parameter]
        return self.run_runner(arguments)

    def run_runner(self, arguments):
        with tempfile.TemporaryFile(mode='w+') as output:
            process = subprocess.Popen(arguments, stdout=output, stderr=subprocess.STDOUT)
            try:
                return_code = process.wait(timeout=40)
            except subprocess.TimeoutExpired:
                process.kill()
                process.wait()
                output.seek(0)
                self.fail('Runner did not exit within 40 seconds:\n' + output.read())
            output.seek(0)
            return subprocess.CompletedProcess(arguments, return_code, output.read())

    def run_xyz(self, target=(0.0073500433, 0.0038391054, 0.4310989486), execute=False,
                extra_parameters=()):
        description_path = (Path(get_package_share_directory('dofbot_description')) /
                            'urdf' / 'dofbot.urdf')
        self.description_publisher = self.arm.create_publisher(
            String, 'robot_description', QoSProfile(depth=1, durability=DurabilityPolicy.TRANSIENT_LOCAL))
        self.description_publisher.publish(String(data=description_path.read_text()))
        arguments = [
            runner_path, '--ros-args', '-p', 'target_frame:=grasp_contact_frame',
            '-p', f'target_xyz:=[{target[0]},{target[1]},{target[2]}]',
            '-p', f'execute:={str(execute).lower()}',
            '-p', 'position_tolerance_degrees:=0.25',
        ]
        for parameter in extra_parameters:
            arguments += ['-p', parameter]
        return self.run_runner(arguments)

    def run_continuous_pose(self, targets=(0.07, -0.06, 0.05, -0.04, 0.03, -0.05), execute=True):
        values = ','.join(str(value) for value in targets)
        return self.run_runner([
            runner_path, '--ros-args', '-p', 'pose_name:=continuous_test',
            '-p', f'poses.continuous_test:=[{values}]',
            '-p', 'continuous_motion:=true', '-p', 'position_tolerance_degrees:=0.1',
            '-p', f'execute:={str(execute).lower()}',
        ])

    def test_continuous_coordinates_six_joints_without_intermediate_stops(self):
        self.arm.enable_continuous_motion()
        targets = (0.07, -0.06, 0.05, -0.04, 0.03, -0.05)
        result = self.run_continuous_pose(targets)
        self.assertEqual(result.returncode, 0, result.stdout)
        self.assertEqual(self.arm.rejected_times, [])
        self.assertGreater(len(self.arm.sent_commands), 5)
        intervals = [later - earlier for earlier, later in
                     zip(self.arm.sent_times, self.arm.sent_times[1:])]
        self.assertTrue(all(0.5 <= interval < 1.0 for interval in intervals), intervals)
        fractions = []
        unfinished_steps = 0
        for index, (measured, command) in enumerate(self.arm.command_measurements):
            fraction = command.position[0] / targets[0]
            fractions.append(fraction)
            self.assertEqual(command.name, self.arm.names)
            for joint_index, (position, target) in enumerate(zip(command.position, targets)):
                self.assertAlmostEqual(position / target, fraction, places=10)
                scale = (172.12 - 0.82) / 1.54 if joint_index == 5 else 180.0 / math.pi
                limit = 4.5 if joint_index == 5 else 2.5
                self.assertLessEqual(abs(position - measured[joint_index]) * scale, limit)
            if index > 0 and abs(measured[0] - self.arm.sent_commands[index - 1].position[0]) > 0.0001:
                unfinished_steps += 1
        self.assertEqual(fractions, sorted(fractions))
        self.assertEqual(list(self.arm.sent_commands[-1].position), list(targets))
        self.assertGreater(unfinished_steps, 2)
        for measured, target in zip(self.arm.positions, targets):
            self.assertAlmostEqual(measured, target, places=5)

    def test_continuous_stall_stops_before_an_excessive_command(self):
        self.arm.enable_continuous_motion()
        self.arm.move = False
        result = self.run_continuous_pose((0.14, 0.0, 0.0, 0.0, 0.0, 0.0))
        self.assertNotEqual(result.returncode, 0, result.stdout)
        self.assertIn('Continuous tracking limit exceeded: joint=arm_joint1', result.stdout)
        self.assertIn('no retry sent', result.stdout)
        for command in self.arm.sent_commands:
            self.assertLessEqual(abs(command.position[0]) * 180.0 / math.pi, 2.5)

    def test_continuous_final_arrival_requires_measured_motion(self):
        self.arm.enable_continuous_motion()
        self.arm.move = False
        result = self.run_continuous_pose((0.03, 0.0, 0.0, 0.0, 0.0, 0.0))
        self.assertNotEqual(result.returncode, 0, result.stdout)
        self.assertIn('Continuous trajectory final target timed out', result.stdout)
        final_commands = [command for command in self.arm.sent_commands if command.position[0] == 0.03]
        self.assertEqual(len(final_commands), 1)

    def test_continuous_missing_confirmation_stops_after_one_command(self):
        self.arm.enable_continuous_motion()
        self.arm.confirmation_mode = 'missing'
        result = self.run_continuous_pose()
        self.assertNotEqual(result.returncode, 0, result.stdout)
        self.assertIn('Command transmission confirmation timed out', result.stdout)
        self.assertEqual(len(self.arm.received_times), 1)

    def test_continuous_requires_a_compatible_driver_before_any_command(self):
        result = self.run_continuous_pose()
        self.assertNotEqual(result.returncode, 0, result.stdout)
        self.assertIn('requires exactly one driver mode publisher', result.stdout)
        self.assertEqual(self.arm.received_times, [])
        self.arm.enable_continuous_motion()
        self.arm.mode_publisher.publish(Bool(data=False))
        result = self.run_continuous_pose()
        self.assertNotEqual(result.returncode, 0, result.stdout)
        self.assertIn('requires continuous_motion:=true on the driver', result.stdout)
        self.assertEqual(self.arm.received_times, [])

    def test_continuous_preview_does_not_require_a_driver_or_send_commands(self):
        result = self.run_continuous_pose(execute=False)
        self.assertEqual(result.returncode, 0, result.stdout)
        self.assertIn('Continuous trajectory planned', result.stdout)
        self.assertEqual(self.arm.received_times, [])

    def test_continuous_gripper_moves_only_the_gripper(self):
        self.arm.enable_continuous_motion()
        arm_positions = (0.1, -0.2, 0.3, -0.4, 0.5)
        self.arm.positions[:5] = arm_positions
        result = self.run_gripper(
            target=-0.05, extra_parameters=('continuous_motion:=true',))
        self.assertEqual(result.returncode, 0, result.stdout)
        self.assertEqual(self.arm.positions[:5], list(arm_positions))
        self.assertTrue(all(command.name == ['grip_joint'] for command in self.arm.sent_commands))

    def test_continuous_fixed_direction_keeps_the_final_geometry_checks(self):
        self.arm.enable_continuous_motion()
        parameters = self.taught_approach() + ('continuous_motion:=true',)
        result = self.run_xyz(target=(0.24281082654399, 0.070177305633513, 0.016846149777103),
                              execute=True, extra_parameters=parameters)
        self.assertEqual(result.returncode, 0, result.stdout)
        self.assertIn('Contact position from joint feedback', result.stdout)
        self.assertIn('Tool direction from joint feedback', result.stdout)
        for command in self.arm.sent_commands:
            self.assertEqual(command.position[4], 0.0004731314237333697)
            self.assertEqual(command.position[5], -1.0648499708114418)

    def test_downward_preview_solves_position_and_direction_without_commands(self):
        self.arm.positions[5] = -0.895
        result = self.run_xyz(target=(0.10, 0.0, 0.02),
                              extra_parameters=('approach_mode:=downward',))
        self.assertEqual(result.returncode, 0, result.stdout)
        self.assertIn('IK contact position', result.stdout)
        self.assertIn('IK tool direction', result.stdout)
        self.assertEqual(self.arm.received_times, [])

    def test_downward_execution_holds_wrist_rotation_and_opening(self):
        self.arm.positions = [0.0, -0.15, -1.45, -1.541592653589793, 0.6, -0.895]
        result = self.run_xyz(target=(0.10, 0.0, 0.02), execute=True,
                              extra_parameters=('approach_mode:=downward',))
        self.assertEqual(result.returncode, 0, result.stdout)
        self.assertIn('Tool direction from joint feedback', result.stdout)
        self.assertGreater(len(self.arm.sent_commands), 0)
        for command in self.arm.sent_commands:
            self.assertEqual(command.position[4], 0.6)
            self.assertEqual(command.position[5], -0.895)

    def test_downward_rejects_a_position_only_reachable_target(self):
        self.arm.positions[5] = -0.895
        result = self.run_xyz(execute=True, extra_parameters=('approach_mode:=downward',))
        self.assertNotEqual(result.returncode, 0, result.stdout)
        self.assertIn('No valid downward IK solution found', result.stdout)
        self.assertEqual(self.arm.received_times, [])

    def test_downward_feedback_check_rejects_direction_despite_position_arrival(self):
        self.arm.positions = [0.0, -0.15, -1.45, -1.530592653589793, 0.0, -0.895]
        disturbed = threading.Event()

        def disturb_after_plan(message):
            if message.name == 'pose_runner' and message.msg.startswith('IK tool direction:') and not disturbed.is_set():
                disturbed.set()
                for joint_index, offset in enumerate((0.0, 0.0138724307, -0.0163213972, 0.00943028353)):
                    self.arm.positions[joint_index] += offset

        self.log_subscription = self.arm.create_subscription(
            Log, 'rosout', disturb_after_plan, rosout_qos)
        result = self.run_xyz(
            target=(0.09970427366563622, -0.0006328786617626552, 0.02237448965604264),
            execute=True, extra_parameters=('approach_mode:=downward', 'position_tolerance_degrees:=1.2'))
        self.assertTrue(disturbed.is_set(), result.stdout)
        self.assertNotEqual(result.returncode, 0, result.stdout)
        self.assertIn('Contact position from joint feedback', result.stdout)
        self.assertIn('Feedback tool direction missed downward by more than 1 degree', result.stdout)
        self.assertEqual(self.arm.received_times, [])

    def test_downward_mode_rejects_unknown_mode_and_incompatible_targets(self):
        for parameters in (
                ('approach_mode:=unknown',),
                ('approach_mode:=downward', 'target_xyz:=[0.1,0.0,0.02]'),
                ('approach_mode:=downward', 'target_frame:=grasp_contact_frame', 'gripper_target:=-0.2'),
                ('approach_mode:=downward', 'target_frame:=grasp_contact_frame',
                 'pick_xyz:=[0.1,0.0,0.02]', 'place_xyz:=[0.12,0.0,0.02]')):
            arguments = [runner_path, '--ros-args']
            for parameter in parameters:
                arguments += ['-p', parameter]
            with self.subTest(parameters=parameters):
                result = self.run_runner(arguments)
                self.assertNotEqual(result.returncode, 0, result.stdout)
                self.assertTrue(any(message in result.stdout for message in (
                    'approach_mode must be', 'Downward approach requires',
                    'requires a single target_xyz')), result.stdout)
                self.assertEqual(self.arm.received_times, [])

    def taught_approach(self):
        self.arm.positions = [0.284171335483804, -0.9767497159342813,
                              -0.8253820698976818, -0.6497384806287982,
                              0.0004731314237333697, -1.0648499708114418]
        return ('approach_mode:=fixed_direction',
                'approach_direction:=[0.61080469809797,0.17004915815026,-0.77330518205547]')

    def test_fixed_direction_preview_uses_taught_direction_without_commands(self):
        parameters = self.taught_approach()
        result = self.run_xyz(target=(0.24281082654399, 0.070177305633513, 0.041846149777103),
                              extra_parameters=parameters)
        self.assertEqual(result.returncode, 0, result.stdout)
        self.assertIn('mode=fixed_direction', result.stdout)
        self.assertIn('IK tool direction', result.stdout)
        self.assertEqual(self.arm.received_times, [])

    def test_fixed_direction_execution_holds_wrist_and_opening(self):
        parameters = self.taught_approach()
        wrist = self.arm.positions[4]
        opening = self.arm.positions[5]
        result = self.run_xyz(target=(0.24281082654399, 0.070177305633513, 0.016846149777103),
                              execute=True, extra_parameters=parameters)
        self.assertEqual(result.returncode, 0, result.stdout)
        self.assertIn('Tool direction from joint feedback', result.stdout)
        self.assertGreater(len(self.arm.sent_commands), 0)
        self.assertEqual(self.arm.rejected_times, [])
        for command in self.arm.sent_commands:
            self.assertEqual(command.position[4], wrist)
            self.assertEqual(command.position[5], opening)

    def test_fixed_direction_unreachable_target_sends_no_commands(self):
        parameters = self.taught_approach()
        result = self.run_xyz(target=(2.0, 2.0, 2.0), execute=True, extra_parameters=parameters)
        self.assertNotEqual(result.returncode, 0, result.stdout)
        self.assertIn('No valid fixed-direction IK solution found', result.stdout)
        self.assertEqual(self.arm.received_times, [])

    def test_fixed_direction_rejects_invalid_configuration_before_commands(self):
        for parameters in (
                ('approach_mode:=fixed_direction', 'target_frame:=grasp_contact_frame',
                 'target_xyz:=[0.1,0.0,0.02]'),
                ('approach_mode:=fixed_direction', 'target_frame:=grasp_contact_frame',
                 'target_xyz:=[0.1,0.0,0.02]', 'approach_direction:=[0.0,0.0,0.0]'),
                ('approach_mode:=fixed_direction', 'target_frame:=grasp_contact_frame',
                 'target_xyz:=[0.1,0.0,0.02]', 'approach_direction:=[0.0,-1.0]'),
                ('approach_mode:=fixed_direction', 'target_xyz:=[0.1,0.0,0.02]',
                 'approach_direction:=[0.0,0.0,-1.0]'),
                ('approach_mode:=downward', 'target_frame:=grasp_contact_frame',
                 'target_xyz:=[0.1,0.0,0.02]', 'approach_direction:=[0.0,0.0,-1.0]'),
                ('approach_mode:=fixed_direction', 'target_frame:=grasp_contact_frame',
                 'gripper_target:=-0.2', 'approach_direction:=[0.0,0.0,-1.0]')):
            arguments = [runner_path, '--ros-args']
            for parameter in parameters:
                arguments += ['-p', parameter]
            with self.subTest(parameters=parameters):
                result = self.run_runner(arguments)
                self.assertNotEqual(result.returncode, 0, result.stdout)
                self.assertTrue(any(message in result.stdout for message in (
                    'approach_direction must', 'Fixed-direction approach requires',
                    'approach_direction requires', 'requires a single target_xyz')), result.stdout)
                self.assertEqual(self.arm.received_times, [])

    def test_fixed_direction_feedback_rejects_wrong_direction_at_arrived_position(self):
        self.arm.positions = [0.0, -0.15, -1.45, -1.530592653589793, 0.0, -0.895]
        disturbed = threading.Event()

        def disturb_after_plan(message):
            if (message.name == 'pose_runner' and
                    message.msg.startswith('IK tool direction:') and not disturbed.is_set()):
                disturbed.set()
                offsets = (0.0, 0.0138724307, -0.0163213972, 0.00943028353)
                for joint_index, offset in enumerate(offsets):
                    self.arm.positions[joint_index] += offset

        self.log_subscription = self.arm.create_subscription(
            Log, 'rosout', disturb_after_plan, rosout_qos)
        result = self.run_xyz(
            target=(0.09970427366563622, -0.0006328786617626552, 0.02237448965604264),
            execute=True, extra_parameters=('approach_mode:=fixed_direction',
                                            'approach_direction:=[0.0,0.0,-1.0]',
                                            'position_tolerance_degrees:=1.2'))
        self.assertTrue(disturbed.is_set(), result.stdout)
        self.assertNotEqual(result.returncode, 0, result.stdout)
        self.assertIn('Contact position from joint feedback', result.stdout)
        self.assertIn('Feedback tool direction missed requested direction by more than 1 degree',
                      result.stdout)
        self.assertEqual(self.arm.received_times, [])

    def test_contact_preview_uses_measured_opening_without_commands(self):
        self.arm.positions[5] = -0.8949375364856976
        result = self.run_xyz()
        self.assertEqual(result.returncode, 0, result.stdout)
        self.assertIn('IK contact position', result.stdout)
        self.assertIn('grip_joint=-0.894938', result.stdout)
        self.assertEqual(self.arm.received_times, [])

    def test_contact_execution_holds_opening_and_checks_contact_arrival(self):
        self.arm.positions[5] = -0.8949375364856976
        result = self.run_xyz(execute=True)
        self.assertEqual(result.returncode, 0, result.stdout)
        self.assertIn('Contact position from joint feedback', result.stdout)
        self.assertGreater(len(self.arm.sent_commands), 1)
        self.assertEqual(self.arm.rejected_times, [])
        for command in self.arm.sent_commands:
            self.assertEqual(command.position[4], 0.0)
            self.assertEqual(command.position[5], -0.8949375364856976)

    def test_contact_position_check_rejects_joint_tolerance_only_arrival(self):
        self.arm.positions[5] = -0.8949375364856976
        self.arm.offset_joint_name = 'arm_joint2'
        self.arm.position_offsets = {index: math.radians(0.24) for index in range(1, 30)}
        result = self.run_xyz(execute=True)
        self.assertNotEqual(result.returncode, 0, result.stdout)
        self.assertIn('Feedback contact position missed the XYZ target', result.stdout)

    def test_unreachable_contact_target_sends_no_commands(self):
        self.arm.positions[5] = -0.8949375364856976
        result = self.run_xyz(target=(2.0, 2.0, 2.0), execute=True)
        self.assertNotEqual(result.returncode, 0, result.stdout)
        self.assertIn('No valid IK solution found', result.stdout)
        self.assertEqual(self.arm.received_times, [])

    def test_contact_requires_fresh_gripper_feedback(self):
        self.arm.feedback_names = self.arm.names[:5]
        result = self.run_xyz()
        self.assertNotEqual(result.returncode, 0, result.stdout)
        self.assertIn('complete fresh feedback', result.stdout)
        self.assertEqual(self.arm.received_times, [])

    def test_contact_frame_rejects_other_modes_and_unknown_frames(self):
        for parameters in (
                ('target_frame:=unknown',),
                ('target_frame:=grasp_contact_frame', 'gripper_target:=-0.2'),
                ('target_frame:=grasp_contact_frame', 'poses.home:=[0.0,0.0,0.0,0.0,0.0,0.0]'),
                ('target_frame:=grasp_contact_frame', 'target_xyz:=[0.02,0.0,0.42]', 'pose_name:=home'),
                ('target_frame:=grasp_contact_frame', 'pick_xyz:=[0.0,0.0,0.4]', 'place_xyz:=[0.02,0.0,0.4]')):
            arguments = [runner_path, '--ros-args']
            for parameter in parameters:
                arguments += ['-p', parameter]
            with self.subTest(parameters=parameters):
                result = self.run_runner(arguments)
                self.assertNotEqual(result.returncode, 0, result.stdout)
                self.assertTrue(any(message in result.stdout for message in (
                    'target_frame must be', 'requires a single target_xyz',
                    'cannot be combined with saved-pose parameters')), result.stdout)
                self.assertEqual(self.arm.received_times, [])

    def test_delayed_transmission_preserves_cooldown_for_multiple_steps(self):
        result = self.run_pose(target=0.1)
        self.assertEqual(result.returncode, 0, result.stdout)
        self.assertEqual(len(self.arm.sent_times), 3, result.stdout)
        self.assertEqual(self.arm.rejected_times, [])
        for previous_sent, next_received in zip(
                self.arm.sent_times, self.arm.received_times[1:]):
            self.assertGreaterEqual(next_received - previous_sent, 2.08)
        self.assertEqual(result.stdout.count('command transmission confirmed'), 3)

    def assert_unmatched_confirmation_stops(self, mode, gripper_only=False):
        self.arm.confirmation_mode = mode
        result = self.run_gripper() if gripper_only else self.run_pose()
        self.assertNotEqual(result.returncode, 0, result.stdout)
        self.assertIn('Command transmission confirmation timed out', result.stdout)
        self.assertEqual(len(self.arm.received_times), 1)
        self.assertNotIn('command transmission confirmed', result.stdout)

    def test_missing_confirmation_stops_without_retry(self):
        self.assert_unmatched_confirmation_stops('missing')

    def test_wrong_stamp_does_not_confirm_another_request(self):
        self.assert_unmatched_confirmation_stops('wrong_stamp')

    def test_wrong_target_does_not_confirm_the_step(self):
        self.assert_unmatched_confirmation_stops('wrong_target')

    def test_wrong_joint_order_does_not_confirm_the_step(self):
        self.assert_unmatched_confirmation_stops('wrong_names')

    def test_preconfirmation_feedback_cannot_complete_the_step(self):
        self.arm.freeze_feedback = True
        result = self.run_pose()
        self.assertNotEqual(result.returncode, 0, result.stdout)
        self.assertIn('command transmission confirmed', result.stdout)
        self.assertNotIn('reached within joint tolerance', result.stdout)
        self.assertEqual(len(self.arm.received_times), 1)

    def test_confirmation_alone_does_not_prove_movement(self):
        self.arm.move = False
        result = self.run_pose()
        self.assertNotEqual(result.returncode, 0, result.stdout)
        self.assertIn('Joint target step timed out after one correction', result.stdout)
        self.assertEqual(len(self.arm.received_times), 2)
        self.assertEqual(result.stdout.count('correction_attempt=1'), 1)

    def test_one_correction_completes_an_undershot_step(self):
        self.arm.position_offsets[1] = -math.radians(0.6)
        result = self.run_pose()
        self.assertEqual(result.returncode, 0, result.stdout)
        self.assertEqual(len(self.arm.sent_commands), 2)
        first, correction = self.arm.sent_commands
        self.assertEqual(first.position, correction.position)
        self.assertNotEqual(first.header.stamp, correction.header.stamp)
        self.assertEqual(result.stdout.count('correction_attempt=1'), 1)
        self.assertGreaterEqual(self.arm.received_times[1] - self.arm.sent_times[0], 4.98)
        self.assertEqual(self.arm.rejected_times, [])

    def run_joint_tolerance_pose(self, joint_name, offset_degrees, extra_parameters=()):
        self.arm.offset_joint_name = joint_name
        self.arm.position_offsets = {index: -math.radians(offset_degrees) for index in (1, 2)}
        targets = [0.0] * 6
        targets[self.arm.names.index(joint_name)] = 0.04
        arguments = [
            runner_path, '--ros-args', '-p', 'pose_name:=joint_tolerance_test',
            '-p', f'poses.joint_tolerance_test:={targets}',
            '-p', 'position_tolerance_degrees:=1.2',
        ]
        for parameter in extra_parameters:
            arguments += ['-p', parameter]
        return self.run_runner(arguments)

    def test_joint2_override_accepts_step_and_final_pose(self):
        result = self.run_joint_tolerance_pose(
            'arm_joint2', 1.4, ('joint2_position_tolerance_degrees:=1.5',))
        self.assertEqual(result.returncode, 0, result.stdout)
        self.assertEqual(len(self.arm.sent_commands), 1, result.stdout)
        self.assertAlmostEqual(self.arm.positions[1], 0.04 - math.radians(1.4))
        self.assertNotIn('correction_attempt', result.stdout)

    def test_joint2_override_does_not_relax_other_joints(self):
        result = self.run_joint_tolerance_pose(
            'arm_joint1', 1.4, ('joint2_position_tolerance_degrees:=1.5',))
        self.assertNotEqual(result.returncode, 0, result.stdout)
        self.assertIn('Joint target step timed out after one correction', result.stdout)
        self.assertEqual(len(self.arm.sent_commands), 2)

    def test_joint2_override_rejects_error_above_its_tolerance(self):
        result = self.run_joint_tolerance_pose(
            'arm_joint2', 1.6, ('joint2_position_tolerance_degrees:=1.5',))
        self.assertNotEqual(result.returncode, 0, result.stdout)
        self.assertIn('Joint target step timed out after one correction', result.stdout)
        self.assertEqual(len(self.arm.sent_commands), 2)

    def test_joint2_without_override_keeps_general_tolerance(self):
        result = self.run_joint_tolerance_pose('arm_joint2', 1.4)
        self.assertNotEqual(result.returncode, 0, result.stdout)
        self.assertIn('Joint target step timed out after one correction', result.stdout)
        self.assertEqual(len(self.arm.sent_commands), 2)

    def test_invalid_joint_tolerances_are_rejected_before_commands(self):
        for parameter in (
                'joint2_position_tolerance_degrees:=0.0',
                'joint2_position_tolerance_degrees:=-0.1',
                'joint2_position_tolerance_degrees:=2.01',
                'joint2_position_tolerance_degrees:=.nan',
                'joint2_position_tolerance_degrees:=.inf',
                'position_tolerance_degrees:=2.01'):
            with self.subTest(parameter=parameter):
                result = self.run_joint_tolerance_pose('arm_joint2', 0.0, (parameter,))
                self.assertNotEqual(result.returncode, 0, result.stdout)
                self.assertIn('tolerance_degrees must be', result.stdout)
                self.assertEqual(self.arm.received_times, [])

    def run_default_tolerance_pose(self, joint_name, offset_degrees):
        self.arm.positions = [0.0] * 6
        self.arm.sent_commands.clear()
        self.arm.received_times.clear()
        self.arm.sent_times.clear()
        self.arm.offset_joint_name = joint_name
        joint_index = self.arm.names.index(joint_name)
        target = -0.04 if joint_index == 5 else 0.04
        offset = (offset_degrees * 1.54 / 171.3 if joint_index == 5
                  else -math.radians(offset_degrees))
        self.arm.position_offsets = {1: offset, 2: offset}
        targets = [0.0] * 6
        targets[joint_index] = target
        return self.run_runner([
            runner_path, '--ros-args', '-p', 'pose_name:=default_tolerance_test',
            '-p', f'poses.default_tolerance_test:={targets}',
        ])

    def test_default_two_degree_tolerance_accepts_all_six_joints(self):
        for joint_name in self.arm.names:
            with self.subTest(joint_name=joint_name):
                result = self.run_default_tolerance_pose(joint_name, 1.8)
                self.assertEqual(result.returncode, 0, result.stdout)
                self.assertEqual(len(self.arm.sent_commands), 1, result.stdout)
                self.assertIn('default=2.000 degrees, arm_joint2=2.000 degrees', result.stdout)
                self.assertNotIn('correction_attempt', result.stdout)

    def test_default_two_degree_tolerance_rejects_larger_error(self):
        for joint_name in ('arm_joint3', 'grip_joint'):
            with self.subTest(joint_name=joint_name):
                result = self.run_default_tolerance_pose(joint_name, 2.1)
                self.assertNotEqual(result.returncode, 0, result.stdout)
                self.assertIn('Joint target step timed out after one correction', result.stdout)
                self.assertEqual(len(self.arm.sent_commands), 2, result.stdout)
                diagnostic_lines = [line.split('Joint step missed: ', 1)[1]
                                    for line in result.stdout.splitlines()
                                    if 'Joint step missed: ' in line]
                self.assertEqual(len(diagnostic_lines), 2, result.stdout)
                joint_index = self.arm.names.index(joint_name)
                target = self.arm.sent_commands[-1].position[joint_index]
                measured = self.arm.positions[joint_index]
                expected = (f'joint={joint_name}, target={target:.6f} rad, '
                            f'measured={measured:.6f} rad, error=2.100 servo degrees, '
                            'tolerance=2.000 servo degrees, '
                            'feedback_after_confirmation=true, phase=')
                self.assertEqual(diagnostic_lines, [expected + 'before_correction',
                                                    expected + 'after_correction'])

    def test_correction_allowance_resets_only_for_a_new_step(self):
        self.arm.position_offsets = {index: -math.radians(0.6) for index in (1, 3, 5)}
        result = self.run_pose(target=0.1)
        self.assertEqual(result.returncode, 0, result.stdout)
        self.assertEqual(len(self.arm.sent_commands), 6, result.stdout)
        self.assertEqual(result.stdout.count('correction_attempt=1'), 3)
        self.assertEqual(self.arm.rejected_times, [])
        for first, correction in zip(self.arm.sent_commands[::2], self.arm.sent_commands[1::2]):
            self.assertEqual(first.position, correction.position)
        for command, previous in zip(self.arm.sent_commands[1:], self.arm.sent_commands):
            self.assertLessEqual(abs(command.position[0] - previous.position[0]),
                                 math.radians(2.5) + 1e-12)

    def test_correction_outside_step_limit_stops_without_resending(self):
        self.arm.position_offsets[1] = -math.radians(3.0)
        result = self.run_pose()
        self.assertNotEqual(result.returncode, 0, result.stdout)
        self.assertIn('correction exceeds step limits', result.stdout)
        self.assertEqual(len(self.arm.sent_commands), 1)
        self.assertNotIn('correction_attempt=1', result.stdout)

    def test_unconfirmed_correction_stops_without_another_attempt(self):
        self.arm.position_offsets[1] = -math.radians(0.6)
        self.arm.unconfirmed_commands.add(2)
        result = self.run_pose()
        self.assertNotEqual(result.returncode, 0, result.stdout)
        self.assertIn('Command transmission confirmation timed out', result.stdout)
        self.assertEqual(len(self.arm.sent_commands), 2)
        self.assertEqual(result.stdout.count('command transmission confirmed'), 1)

    def test_calculation_only_needs_no_confirmation_publisher(self):
        self.arm.destroy_publisher(self.arm.sent_publisher)
        result = self.run_pose(execute=False)
        self.assertEqual(result.returncode, 0, result.stdout)
        self.assertIn('execution disabled', result.stdout)
        self.assertEqual(self.arm.received_times, [])

    def test_execution_requires_a_confirmation_publisher(self):
        self.arm.destroy_publisher(self.arm.sent_publisher)
        result = self.run_pose()
        self.assertNotEqual(result.returncode, 0, result.stdout)
        self.assertIn('transmission confirmation publisher', result.stdout)
        self.assertEqual(self.arm.received_times, [])

    def test_gripper_only_feedback_completes_bounded_steps_without_arm_commands(self):
        self.arm.feedback_names = ['grip_joint']
        self.arm.positions = [0.2, -0.3, 0.4, -0.2, 0.1, -0.1]
        initial_positions = list(self.arm.positions)
        result = self.run_gripper(target=-0.22)
        self.assertEqual(result.returncode, 0, result.stdout)
        self.assertIn('Gripper target reached within joint tolerance', result.stdout)
        self.assertEqual(len(self.arm.sent_commands), 3, result.stdout)
        self.assertEqual(self.arm.positions[:5], initial_positions[:5])
        previous_position = initial_positions[5]
        for command in self.arm.sent_commands:
            self.assertEqual(command.name, ['grip_joint'])
            self.assertEqual(len(command.position), 1)
            self.assertEqual(len(command.velocity), 0)
            self.assertEqual(len(command.effort), 0)
            step_degrees = abs(command.position[0] - previous_position) * 171.3 / 1.54
            self.assertLessEqual(step_degrees, 4.5 + 1e-12)
            previous_position = command.position[0]
        self.assertAlmostEqual(self.arm.positions[5], -0.22)
        for previous_sent, next_received in zip(
                self.arm.sent_times, self.arm.received_times[1:]):
            self.assertGreaterEqual(next_received - previous_sent, 2.08)
        self.assertEqual(self.arm.rejected_times, [])

    def test_gripper_only_ignores_invalid_uncommanded_arm_feedback(self):
        self.arm.positions[0] = float('nan')
        self.arm.positions[1] = 2.0
        result = self.run_gripper()
        self.assertEqual(result.returncode, 0, result.stdout)
        self.assertEqual(len(self.arm.sent_commands), 1)
        self.assertTrue(math.isnan(self.arm.positions[0]))
        self.assertEqual(self.arm.positions[1], 2.0)
        self.assertEqual(self.arm.sent_commands[0].name, ['grip_joint'])

    def test_gripper_reset_reading_plans_from_the_actual_servo_angle(self):
        self.arm.feedback_names = ['grip_joint']
        servo_degrees = (3040 - 900) * 180.0 / 2200.0
        self.arm.positions[5] = (servo_degrees - 172.12) * 1.54 / 171.3
        previous_position = self.arm.positions[5]
        result = self.run_gripper(
            target=-0.32, extra_parameters=('position_tolerance_degrees:=1.0',))
        self.assertEqual(result.returncode, 0, result.stdout)
        self.assertEqual(len(self.arm.sent_commands), 9, result.stdout)
        for command in self.arm.sent_commands:
            self.assertEqual(command.name, ['grip_joint'])
            self.assertGreaterEqual(command.position[0], -1.54)
            self.assertLessEqual(command.position[0], 0.0)
            step_degrees = abs(command.position[0] - previous_position) * 171.3 / 1.54
            self.assertLessEqual(step_degrees, 4.5 + 1e-12)
            previous_position = command.position[0]
        first_servo_degrees = 172.12 + self.arm.sent_commands[0].position[0] * 171.3 / 1.54
        self.assertAlmostEqual(first_servo_degrees, servo_degrees - 4.5)
        self.assertAlmostEqual(self.arm.positions[5], -0.32)

    def test_gripper_reset_reading_can_return_to_the_closed_reference(self):
        self.arm.positions[5] = (175.09 - 172.12) * 1.54 / 171.3
        result = self.run_gripper(target=0.0)
        self.assertEqual(result.returncode, 0, result.stdout)
        self.assertEqual(len(self.arm.sent_commands), 1)
        self.assertEqual(self.arm.sent_commands[0].name, ['grip_joint'])
        self.assertEqual(list(self.arm.sent_commands[0].position), [0.0])

    def test_gripper_reading_beyond_open_reference_allows_bounded_inward_steps(self):
        self.arm.positions[5] = -172.12 * 1.54 / 171.3
        result = self.run_gripper(target=-1.50)
        self.assertEqual(result.returncode, 0, result.stdout)
        self.assertEqual(len(self.arm.sent_commands), 2)
        for command in self.arm.sent_commands:
            self.assertGreaterEqual(command.position[0], -1.54)
            self.assertLessEqual(command.position[0], 0.0)

    def test_gripper_stops_when_command_range_is_beyond_one_bounded_step(self):
        self.arm.positions[5] = (180.0 - 172.12) * 1.54 / 171.3
        result = self.run_gripper(target=-0.32)
        self.assertNotEqual(result.returncode, 0, result.stdout)
        self.assertIn('bounded step', result.stdout)
        self.assertIn('command range', result.stdout)
        self.assertEqual(self.arm.received_times, [])

    def test_gripper_feedback_outside_physical_range_is_rejected(self):
        for servo_degrees in (-0.1, 180.1):
            with self.subTest(servo_degrees=servo_degrees):
                self.arm.positions[5] = (servo_degrees - 172.12) * 1.54 / 171.3
                result = self.run_gripper()
                self.assertNotEqual(result.returncode, 0, result.stdout)
                self.assertIn('fresh gripper feedback', result.stdout)
                self.assertEqual(self.arm.received_times, [])

    def test_gripper_review_defaults_to_no_movement_without_command_endpoints(self):
        self.arm.feedback_names = ['grip_joint']
        self.arm.destroy_publisher(self.arm.sent_publisher)
        self.arm.destroy_subscription(self.arm.command_subscription)
        result = self.run_gripper(execute=None)
        self.assertEqual(result.returncode, 0, result.stdout)
        self.assertIn('execution disabled', result.stdout)
        self.assertEqual(self.arm.received_times, [])

    def test_gripper_requires_gripper_feedback_despite_fresh_arm_feedback(self):
        self.arm.feedback_names = self.arm.names[:5]
        result = self.run_gripper()
        self.assertNotEqual(result.returncode, 0, result.stdout)
        self.assertIn('fresh gripper feedback', result.stdout)
        self.assertEqual(self.arm.received_times, [])

    def test_gripper_missing_confirmation_stops_without_retry(self):
        self.assert_unmatched_confirmation_stops('missing', gripper_only=True)

    def test_gripper_wrong_confirmation_stamp_is_rejected(self):
        self.assert_unmatched_confirmation_stops('wrong_stamp', gripper_only=True)

    def test_gripper_wrong_confirmation_target_is_rejected(self):
        self.assert_unmatched_confirmation_stops('wrong_target', gripper_only=True)

    def test_gripper_confirmation_for_another_joint_is_rejected(self):
        self.assert_unmatched_confirmation_stops('wrong_names', gripper_only=True)

    def test_gripper_six_joint_confirmation_is_rejected(self):
        self.assert_unmatched_confirmation_stops('extra_joints', gripper_only=True)

    def test_gripper_stale_feedback_stops_without_retry(self):
        self.arm.feedback_names = ['grip_joint']
        self.arm.freeze_feedback = True
        result = self.run_gripper()
        self.assertNotEqual(result.returncode, 0, result.stdout)
        self.assertIn('Gripper feedback became stale', result.stdout)
        self.assertEqual(len(self.arm.received_times), 1)

    def test_gripper_one_correction_keeps_the_same_single_joint_target(self):
        self.arm.feedback_names = ['grip_joint']
        self.arm.offset_joint_name = 'grip_joint'
        self.arm.position_offsets[1] = 0.6 * 1.54 / 171.3
        result = self.run_gripper()
        self.assertEqual(result.returncode, 0, result.stdout)
        self.assertEqual(len(self.arm.sent_commands), 2)
        first, correction = self.arm.sent_commands
        self.assertEqual(first.name, ['grip_joint'])
        self.assertEqual(correction.name, first.name)
        self.assertEqual(correction.position, first.position)
        self.assertNotEqual(correction.header.stamp, first.header.stamp)
        self.assertEqual(result.stdout.count('correction_attempt=1'), 1)

    def test_gripper_confirmation_without_movement_stops_after_one_correction(self):
        self.arm.move = False
        result = self.run_gripper()
        self.assertNotEqual(result.returncode, 0, result.stdout)
        self.assertIn('Joint target step timed out after one correction', result.stdout)
        self.assertEqual(len(self.arm.sent_commands), 2)
        self.assertTrue(all(command.name == ['grip_joint'] for command in self.arm.sent_commands))

    def test_gripper_invalid_targets_are_rejected_before_publication(self):
        for target in (-2.0, 0.1, '.nan', '.inf'):
            with self.subTest(target=target):
                result = self.run_gripper(target=target)
                self.assertNotEqual(result.returncode, 0, result.stdout)
                self.assertEqual(self.arm.received_times, [])

    def test_gripper_conflicting_modes_are_rejected_before_publication(self):
        for parameters in (
                ('pose_name:=home',),
                ('poses.home:=[0.0,0.0,0.0,0.0,0.0,0.0]',),
                ('target_xyz:=[0.0,0.0,0.4]',),
                ('pick_xyz:=[0.0,0.0,0.4]', 'place_xyz:=[0.02,0.0,0.4]')):
            with self.subTest(parameters=parameters):
                result = self.run_gripper(extra_parameters=parameters)
                self.assertNotEqual(result.returncode, 0, result.stdout)
                self.assertIn('cannot be combined', result.stdout)
                self.assertEqual(self.arm.received_times, [])


if __name__ == '__main__':
    unittest.main(verbosity=2)
