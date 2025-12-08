#!/usr/bin/env python

import rclpy
from  drive_control import explicit_logic, drive_pid
from rclpy.node import Node
from sensor_msgs.msg import Joy
from drive_control_interfaces.msg import DriveData, DriveEncoderData


class DriveController(Node):
    def __init__(self):
        super().__init__('drive_controller')

        # Create a subscription to Joystick 
        self.joystick_subscription = self.create_subscription(Joy, '/joy', self.joystick_callback, 10)

        # Create a subscription to Encoder values (rn just magnetic)
        self.encoder_subscription = self.create_subscription(DriveEncoderData, '/drive_encoders', self.encoder_callback, 10)

        # Create a publisher for Drive Commands
        self.drive_publisher = self.create_publisher(DriveData,'/drive_commands',10)

        # self.sys_check_toggle = False

        # creating 4 instances for each wheel
        self.FR_pid = drive_pid.PositionController()
        self.BR_pid = drive_pid.PositionController()
        self.BL_pid = drive_pid.PositionController()
        self.FL_pid = drive_pid.PositionController()

        # DO IN RECEIVING CODE
        # self.FR_mag_offset = 0
        # self.BR_mag_offset = 0
        # self.BL_mag_offset = 0
        # self.FL_mag_offset = 0

        self.pids = [self.FR_pid, self.BR_pid, self.BL_pid, self.FL_pid]

        self.velocities = [0] * 4



        left_hor = 0.5
        left_ver = 0.5

        v = explicit_logic.VroomVroom()
        explicit_values = v.smooooth_operatorrrr(left_hor, left_ver)

        # #setting the angles for each of the 4 wheels
        # for i in range(4):
        #     self.pids[i].set_position(explicit_values[0][i])
        self.FR_pid.set_position(explicit_values[0][0])
        self.BR_pid.set_position(explicit_values[0][1])
        self.BL_pid.set_position(explicit_values[0][2])
        self.FL_pid.set_position(explicit_values[0][3])

        self.velocities = explicit_values[1]

    def sign(self, x):
        if x > 0:
            return 1
        else:
            return 0

    def joystick_callback(self, joy_val):
        # Extract Joystick values
        left_hor = joy_val.axes[0]
        left_ver = joy_val.axes[1]

        v = explicit_logic.VroomVroom()

        # Return: Returns 2 lists of angles and velocities respectively. The order of motors is in: Front Right, Back Right, Back Left, Front Left
        explicit_values = v.smooooth_operatorrrr(left_hor, left_ver)

        # #setting the angles for each of the 4 wheels
        # for i in range(4):
        #     self.pids[i].set_position(explicit_values[0][i])
        self.FR_pid.set_position(explicit_values[0][0])
        self.BR_pid.set_position(explicit_values[0][1])
        self.BL_pid.set_position(explicit_values[0][2])
        self.FL_pid.set_position(explicit_values[0][3])

        self.velocities = explicit_values[1]

        self.send()

    def encoder_callback(self, enc_data):
        # print("hmm")
        for i in range(4):
            # print(i)
            # print(len(self.pids))
            self.pids[i].add_position_feedback(enc_data.magnetic_data[0])

        self.send()

    def send(self):
        drive_data = DriveData()
        full_arr = []
        for i in range(4):
            full_arr.append(self.velocities[i])
            full_arr.append(self.pids[i].current_output)

        print(full_arr)

        drive_data.dir_arr = [self.sign(x) for x in full_arr]
        drive_data.pwm_arr = [min(255, int(abs(x))) for x in full_arr]



        self.drive_publisher.publish(drive_data)

def main(args=None):
    rclpy.init(args=args)

    drive_control = DriveController()

    try:
        rclpy.spin(drive_control)
    except KeyboardInterrupt:
        print("Shutting down due to KeyboardInterrupt")
    except Exception as e:
        print(f"Exception {e}")
    finally:
        drive_control.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()
