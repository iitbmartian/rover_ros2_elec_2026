#!/usr/bin/env python

import rclpy
from  drive_control import explicit_logic, drive_pid
from rclpy.node import Node
from sensor_msgs.msg import Joy
from drive_control_interfaces.msg import DriveData, TelemetryData
import matplotlib.pyplot as plt

class DriveController(Node):
    def __init__(self):
        super().__init__('drive_controller')

        # Create a subscription to Joystick 
        self.joystick_subscription = self.create_subscription(Joy, '/joy', self.joystick_callback, 10)

        # Create a subscription to Encoder values (rn just magnetic)
        self.encoder_subscription = self.create_subscription(TelemetryData, '/telemetry', self.encoder_callback, 10)

        # Create a publisher for Drive Commands
        self.drive_publisher = self.create_publisher(DriveData,'/drive_commands',10)

        # self.sys_check_toggle = False

        self.vroomer = explicit_logic.VroomVroom()

        # creating 4 instances for each wheel
        self.FR_ppid = drive_pid.PositionController(Kp=7)
        self.FL_ppid = drive_pid.PositionController(Kp=10)
        self.BL_ppid = drive_pid.PositionController(Kp=9)
        self.BR_ppid = drive_pid.PositionController(Kp=8)

        self.FR_vpid = drive_pid.VelocityController()
        self.FL_vpid = drive_pid.VelocityController()
        self.BL_vpid = drive_pid.VelocityController()
        self.BR_vpid = drive_pid.VelocityController()

        self.FR_mag_offset = 355.0
        self.FL_mag_offset = 238.35
        self.BL_mag_offset = 183.33
        self.BR_mag_offset = 104.32

        self.ppids = [self.FR_ppid, self.FL_ppid, self.BL_ppid, self.BR_ppid]
        self.vpids = [self.FR_vpid, self.FL_vpid, self.BL_vpid, self.BR_vpid]
        self.mag_offsets = [self.FR_mag_offset, self.FL_mag_offset, self.BL_mag_offset, self.BR_mag_offset]

        self.steer_input_direction_flip = [False, False, False, False]
        self.vroom_input_direction_flip = [False, False, True, False]

        self.steer_output_direction_flip = [True, False, True, False]
        self.vroom_output_direction_flip = [True, True, True, True]

        self.velocities = [0] * 4
        global actual_vel_history, target_vel_history

        self.error_history = []
        actual_vel_history = []
        target_vel_history = []

        # left_hor = 0.5
        # left_ver = 0.5
        #
        # explicit_values = self.vroomer.smooooth_operatorrrr(left_hor, left_ver)
        #
        # # #setting the angles for each of the 4 wheels
        # # for i in range(4):
        # #     self.pids[i].set_position(explicit_values[0][i])
        # self.FR_pid.set_position(explicit_values[0][0])
        # self.BR_pid.set_position(explicit_values[0][1])
        # self.BL_pid.set_position(explicit_values[0][2])
        # self.FL_pid.set_position(explicit_values[0][3])
        #
        # self.velocities = explicit_values[1]

    def get_direction(self, x):
        if x > 0:
            return 1
        elif x < 0:
            return -1
        else:
            return 0

    def joystick_callback(self, joy_val):
        # Extract Joystick values
        left_hor = -joy_val.axes[0]
        L2_val = (1 - joy_val.axes[2]) / 2
        R2_val = (1 - joy_val.axes[5]) / 2
        left_ver = R2_val - L2_val


        # Return: Returns 2 lists of angles and velocities respectively. The order of motors is in: Front Right, Back Right, Back Left, Front Left
        explicit_values = self.vroomer.smooooth_operatorrrr(left_hor, left_ver)
        target_vel_history.append((self.get_clock().now(), explicit_values[1][0]))
        print(f"Calculated explicit values: {explicit_values}")

        # #setting the angles for each of the 4 wheels
        for i in range(4):
            self.ppids[i].set_position(explicit_values[0][i])
            self.vpids[i].set_velocity(explicit_values[1][i])

            print(f"desired angle {i}: {explicit_values[0][i]}")
            print(f"desired velocity {i}: {explicit_values[1][i]}")

        # TODO put feedback
        self.velocities = explicit_values[1]

        self.send()

    def format_magnetic_angle(self, angle, motor_index):
        real_angle = angle - self.mag_offsets[motor_index]

        while real_angle > 180:
            real_angle -= 360

        while real_angle < -180:
            real_angle += 360

        return real_angle

    def encoder_callback(self, telemetry_data):
        enc_data = telemetry_data.angles
        vel_data = telemetry_data.speed
        actual_vel_history.append((self.get_clock().now(), vel_data[0]))
        for i in range(4):
            magnetic_angle = self.format_magnetic_angle(enc_data[i], i)

            if self.steer_input_direction_flip[i]:
                magnetic_angle = -magnetic_angle

            if self.vroom_input_direction_flip[i]:
                vel_data[i] = -vel_data[i]

            print(f"formatted angle {i}: {magnetic_angle}")
            print(f"actual vel {i}: {vel_data[i]}")

            self.ppids[i].add_position_feedback(magnetic_angle)
            self.vpids[i].add_velocity_feedback(vel_data[i])

        self.send()

    def send(self):
        drive_data = DriveData()
        full_arr = []
        for i in range(4):
            # TODO put feedback
            # v_output = self.vpids[i].current_output
            v_output = self.velocities[i]
            p_output = self.ppids[i].current_output

            if self.vroom_output_direction_flip[i]:
                v_output = -v_output

            if self.steer_output_direction_flip[i]:
                p_output = -p_output

            full_arr.append(v_output)
            full_arr.append(p_output)

        print(full_arr)

        drive_data.pwm = [min(255, int(abs(x))) for x in full_arr]
        drive_data.direction = [self.get_direction(x) for x in full_arr]

        # drive_data.direction = [0, 0]
        # drive_data.pwm = [100, 200]

        self.drive_publisher.publish(drive_data)

def main(args=None):
    actual_vel_history = []
    target_vel_history = []

    rclpy.init(args=args)

    drive_control = DriveController()

    try:
        rclpy.spin(drive_control)
    except KeyboardInterrupt:
        print("Shutting down due to KeyboardInterrupt")
    # except Exception as e:
        # print(f"Exception {e}")
    finally:
        plt.plot(actual_vel_history)
        plt.plot(target_vel_history)
        # plt.show()
        drive_control.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()
