#!/usr/bin/env python

import rclpy
import explicit_logic
import drive_pid
from rclpy.node import Node
from sensor_msgs.msg import Joy
from drive_control_interfaces import DriveData

sys_check_toggle = False

class DriveController(Node):
    def __init__(self):
        super().__init__('drive_controller')

        # Create a subscription to Joystick 
        self.joystick_subscription = self.create_subscription(Joy,'/joy',self.joystick_callback,10)

        # Create a publisher for Drive Commands
        self.drive_publisher = self.create_publisher(DriveData,'drive_commands',10)

        #creating 4 instances for each wheel 
        self.FR_D_pid = drive_pid.VelocityController()
        self.BR_D_pid = drive_pid.VelocityController()
        self.BL_D_pid = drive_pid.VelocityController()   
        self.FL_D_pid = drive_pid.VelocityController()

    def sign(self,x):
        if x > 0:
            return 1
        elif x < 0:
            return -1
        else:
            return 0
    
    def joystick_callback(self,joy_val):
        global sys_check_toggle
        drive_command = DriveData()

        # Extract Joystick values
        left_hor = joy_val.axes[0]
        left_ver = joy_val.axes[1]

        sys_check_trigger = joy_val.axes[6] #for sys_check

        sys_check_toggle = (sys_check_trigger == 1) #True if 1, False if not

        v = explicit_logic.VroomVroom()

        # Return: Returns 2 lists of angles and velocities respectively. The order of motors is in: Front Right, Back Right, Back Left, Front Left
        explicit_values = v.smooooth_operatorrrr(left_hor,left_ver)

        # Getting the angles and the speeds from the explicit_values variable
        drive_command.angle = explicit_values[0] #angle of the explicit rotation motors
        drive_command.speed = explicit_values[1] #speed of each of the drive motors
 
        # Setting the velocities for each of the 4 drive wheels
        self.FR_D_pid.set_velocity(explicit_values[1][0])
        self.BR_D_pid.set_velocity(explicit_values[1][1])
        self.BL_D_pid.set_velocity(explicit_values[1][2])
        self.FL_D_pid.set_velocity(explicit_values[1][3])
        
        # First 4 are of explicit rotation, last 4 of drive motors, each in same order FR BR BL FL
        drive_command.pwm = [0,0,0,0,self.FR_D_pid.current_output, self.BR_D_pid.current_output, self.BL_D_pid.current_output, self.FL_D_pid.current_output] #pwm from pid logic
        drive_command.direction = [self.sign(explicit_values[1][0]),self.sign(explicit_values[1][1]),self.sign(explicit_values[1][2]),self.sign(explicit_values[1][3])]
        drive_command.sys_check = sys_check_toggle

        self.drive_publisher.publish(drive_command)
        self.get_logger().info(f'Published drive command - Speed: {list(drive_command.speed)}, Direction: {list(drive_command.direction)}, PWM: {list(drive_command.pwm)}, Angle: {list(drive_command.angle)}, System Check Request: {drive_command.sys_check}')


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
