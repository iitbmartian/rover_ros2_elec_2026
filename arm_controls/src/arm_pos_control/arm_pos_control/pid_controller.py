import rclpy
from rclpy.action import ActionServer, GoalResponse
from rclpy.action.server import ServerGoalHandle
from rclpy.callback_groups import ReentrantCallbackGroup
from rclpy.node import Node
from rclpy.executors import MultiThreadedExecutor

import ast
import threading
import math
import time
import numpy as np
from numpy.typing import NDArray
from typing import List, Tuple

from pos_control_interfaces.action import Trajectory
from msg_interfaces.msg import EncoderArm, ArmEndMotion
from geometry_msgs.msg import Point

from arm_pos_control.calculator import IKHandler
from arm_pos_control.min_jerk import min_jerk

MAX_QUEUE_SIZE = 10
SAMPLE_FREQ = 100  # Hz

# TODO: maybe a launch node

class TrajectoryActionServer(Node):

    def __init__(self):
        super().__init__('trajectory_action_server')

        self.pwm_publisher = self.create_publisher(
            ArmEndMotion, 'arm_commands', MAX_QUEUE_SIZE)
        self.receive_encoder = self.create_subscription(
            EncoderArm, '/encoder_arm', self.encoderCallback, MAX_QUEUE_SIZE)

        self.action_server = ActionServer(
            self,
            Trajectory,
            'pid_controller_actions',
            self.actionCallback,
            goal_callback=self.goalCallback,
            callback_group=ReentrantCallbackGroup()
        )

        self.lock = threading.Lock()
        self.get_logger().info("Started PID Controller")

        self.ik_handler = IKHandler()
        self.q = [0, 1.57, 1.57]  # current base, shoulder, elbow "angles"

        self.end_eff_pos: NDArray[np.float64] = np.array(
            [self.ik_handler.meow_nigga+self.ik_handler.meow_e, self.ik_handler.meow_s], dtype=np.float64)

        self.stop = False
        self.traj_pos: NDArray[np.float64] = []  # numpy array
        self.traj_times: NDArray[np.float64] = []  # numpy array

    @staticmethod
    def arrToPoint(position: NDArray[np.float64]) -> Point:
        outp = Point()
        outp.x = float(position[0])
        outp.y = float(position[1])
        outp.z = 0.0
        return outp

    @staticmethod
    def getTrajectory(current_pos: NDArray[np.float64], waypoints: List[List[float]], timepoints: List[float]) -> Tuple[NDArray[np.float64], NDArray[np.float64]]:
        pos = np.array(waypoints, dtype=np.float64)
        pos = np.insert(pos, 0, current_pos, axis=0)

        total_time = timepoints[-1]
        tp = timepoints
        tp.pop()
        tp = np.array(tp, np.float64) * SAMPLE_FREQ

        dur = math.ceil(total_time * SAMPLE_FREQ) + 1
        traj, psg = min_jerk(pos, dur, None, None, tp)

        t = np.full(dur-1, total_time/(dur-1), dtype=np.float64)
        return traj[1:], t

    def encoderCallback(self, msg: EncoderArm):
        # min - 10698, home - 65536, max - 117678
        current_base_pos = msg.arm_node2[0]
        # min - -8650, home - 0, max - 8680
        current_shoulder_pos = msg.arm_node0[0]
        # min - -9361, home - 0, max - 4680
        current_elbow_pos = msg.arm_node1[0]

        # Convert from continuous joint angles to desired encoder positions
        # motor_side - 0.24, home (middle) - 0.0, other_side - -0.21

        with self.lock:
            self.q[0] = -0.21 + ((0.45 * (current_base_pos)) / 117678)
            self.q[1] = math.pi / 2 + current_shoulder_pos / 5670.3
            self.q[2] = math.pi / 2 - current_elbow_pos / 5670.3

        end_eff_x, end_eff_y = self.ik_handler.K(self.q[1], self.q[2])
        self.end_eff_pos = np.array([end_eff_x, end_eff_y], dtype=np.float64)

        if self.q[1] > 0.75 * math.pi or self.q[1] < 0.25 * math.pi or self.q[2] < 0.25 * math.pi:
            self.stop = True

    def goalCallback(self, goal_request: Trajectory.Goal) -> GoalResponse:
        inp = goal_request.command.data
        try:
            waypoints, timepoints = ast.literal_eval(inp)
            if len(waypoints) % 2:  # odd number of coordinates
                return GoalResponse.REJECT

            if len(waypoints)/2 != len(timepoints):  # number of points = number of coordinates
                return GoalResponse.REJECT

        except (ValueError, SyntaxError):
            if inp != "check" and inp != "reset":
                return GoalResponse.REJECT

        # Accept the goal if all checks pass
        self.get_logger().info(f'Goal {inp} ACCEPTED')
        self.stop = False
        return GoalResponse.ACCEPT

    def actionCallback(self, goal_handle: ServerGoalHandle) -> Trajectory.Result:
        """
        Executes the accepted goal.
        This is only called if goal_callback returned ACCEPT.
        """
        inp = goal_handle.request.command.data
        self.get_logger().info(f'Executing goal {inp}')

        result = Trajectory.Result()
        pwm_msg = ArmEndMotion()
        pwm_msg.direction = [0, 0, 0, 0, 0, 0]
        pwm_msg.speed = [0, 0, 0, 0, 0, 0]
        pwm_msg.sys_check = False
        pwm_msg.reset = False

        if inp == "check":
            pwm_msg.sys_check = True
            self.pwm_publisher.publish(pwm_msg)
            result.success = True
            goal_handle.succeed()
            return result

        if inp == "reset":
            pwm_msg.reset = True
            self.pwm_publisher.publish(pwm_msg)
            result.success = True
            goal_handle.succeed()
            return result

        coords, times = ast.literal_eval(f"({inp})")
        coords = [[coords[i], coords[i+1]] for i in range(0, len(coords), 2)]
        self.traj_pos, self.traj_times = self.getTrajectory(
            self.end_eff_pos, coords, times)

        self.publishPWM(goal_handle)

        if not goal_handle.is_active:
            pwm_msg.sys_check = True
            self.pwm_publisher.publish(pwm_msg)
            result.success = False
            return result

        goal_handle.succeed()
        result.success = True
        return result

    def publishPWM(self, goal_handle: ServerGoalHandle):
        pwm_msg = ArmEndMotion()
        for point, delay in zip(self.traj_pos, self.traj_times):
            with self.lock:
                end_eff_copy = np.copy(self.end_eff_pos)
                stop_flag = self.stop

            # publish feedback
            feedback = Trajectory.Feedback()
            feedback.end_effector_pos = self.arrToPoint(end_eff_copy)
            goal_handle.publish_feedback(feedback)

            pwm_msg.direction = [0, 0, 0, 0, 0, 0]
            pwm_msg.speed = [0, 0, 0, 0, 0, 0]
            pwm_msg.sys_check = False
            pwm_msg.reset = False

            if stop_flag:
                self.get_logger().warn('Going out of bounds, aborting goal')
                goal_handle.abort()
                return

            desired_qs, desired_qe = self.ik_handler.IK(*point)
            print(f'Current Angle Values: {self.q[1], self.q[2]}')
            print(f'Desired Angle Values: {desired_qs, desired_qe}')

            with self.lock:
                (pwm_s, dir_s), (pwm_e, dir_e) = self.ik_handler.PIDs(
                    desired_qs, desired_qe, self.q[1], self.q[2])

            if pwm_s < 25:
                pwm_s = 25
            if pwm_e < 25:
                pwm_e = 25

            pwm_msg.direction = [dir_s, dir_e, 0, 0, 0, 0]
            pwm_msg.speed = [pwm_s, pwm_e, 0, 0, 0, 0]

            self.get_logger().info(
                f'PWM: {pwm_msg.speed}\nDirection: {pwm_msg.direction}')
            
            self.pwm_publisher.publish(pwm_msg)

            self.wait(float(delay))

#        self.loopLastPoint(goal_handle)

#    def loopLastPoint(self, goal_handle: ServerGoalHandle):
        self.get_logger().info("Looping last point.")

        last_point = self.traj_pos[-1]
        desired_qs, desired_qe = self.ik_handler.IK(*last_point)
        (pwm_s, dir_s), (pwm_e, dir_e) = self.ik_handler.PIDs(
            desired_qs, desired_qe, self.q[1], self.q[2])
        
        pwm_msg = ArmEndMotion()
        counter = 0

        while (pwm_s > 40 or pwm_e > 40) and counter < 50 :
            # publish feedback
            with self.lock:
                end_eff_copy = np.copy(self.end_eff_pos)
                stop_flag = self.stop

            feedback = Trajectory.Feedback()
            feedback.end_effector_pos = self.arrToPoint(end_eff_copy)
            goal_handle.publish_feedback(feedback)

            pwm_msg.direction = [0, 0, 0, 0, 0, 0]
            pwm_msg.speed = [0, 0, 0, 0, 0, 0]
            pwm_msg.sys_check = False
            pwm_msg.reset = False

            if stop_flag:
                self.get_logger().warn('Going out of bounds, aborting goal')
                goal_handle.abort()
                return

            pwm_msg.speed = [pwm_s, pwm_e, 0, 0, 0, 0]
            pwm_msg.direction = [dir_s, dir_e, 0, 0, 0, 0]

            self.get_logger().info(
                f'PWM: {pwm_msg.speed}\nDirection: {pwm_msg.direction}')
            self.pwm_publisher.publish(pwm_msg)

            self.wait(1 / SAMPLE_FREQ)

            with self.lock:
                (pwm_s_new, dir_s), (pwm_e_new, dir_e) = self.ik_handler.PIDs(
                    desired_qs, desired_qe, self.q[1], self.q[2])
            
            if abs(pwm_s_new - pwm_s) < 2 and abs(pwm_e_new - pwm_e) < 2:
                counter += 1
            
            pwm_s = pwm_s_new
            pwm_e = pwm_e_new
        
        for _ in range(10):
            pwm_msg.direction = [0,0, 0, 0, 0, 0]
            pwm_msg.speed = [0,0, 0, 0, 0, 0]
            pwm_msg.sys_check = False
            pwm_msg.reset = False
            self.pwm_publisher.publish(pwm_msg)

    def wait(self, duration: float):
        """Wait for duration"""
        time.sleep(duration)
        # rate = self.create_rate(SAMPLE_FREQ * 10)  # Check 10 times
        # start_time = self.get_clock().now()

        # while True:
        #     current_time = self.get_clock().now()
        #     elapsed = (current_time - start_time).nanoseconds / 1e9
        #     if elapsed >= duration:
        #         break

        #     rate.sleep()


def main(args=None):
    rclpy.init(args=args)

    action_server = TrajectoryActionServer()

    # Use MultiThreadedExecutor to handle concurrent callbacks
    executor = MultiThreadedExecutor()
    executor.add_node(action_server)

    try:
        executor.spin()
    except KeyboardInterrupt:
        pass
    finally:
        action_server.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()
