#!/usr/bin/env python

import rclpy
from rclpy.node import Node
from sensor_msgs.msg import Joy
from all_interfaces.msg import LinearBaseData

pwm_deadzone = 500 #if pwm calculated < 500 then ignore

class LinearBaseController(Node):
    def __init__(self):
        super().__init__('linear_base_controller')

        # Create a subscription to Joystick 
        self.joystick_subscription = self.create_subscription(Joy,'/joy',self.joystick_callback,10)

        # Create a publisher for Drive Commands
        self.linear_base_publisher = self.create_publisher(LinearBaseData,'/linear_base_commands',10)
    
    def get_pwm(self,x):
        global pwm_deadzone
        pwm_before_deadzone = int(x*4095)
        if(abs(pwm_before_deadzone) < pwm_deadzone):
            return 0
        else:
            return pwm_before_deadzone
    
    def sign(self,x):
        if x > 0:
            return 1
        elif x < 0:
            return -1
        else:
            return 0
    
    def joystick_callback(self,joy_val):
        linear_base_command = LinearBaseData()

        #Extract Joystick Values (right joystick horizontal motion)
        right_hor = joy_val.axes[3]

        pwm = self.get_pwm(right_hor)

        linear_base_command.pwm = pwm
        linear_base_command.speed = 0 #placeholder for now
        linear_base_command.direction = self.sign(pwm)

        self.linear_base_publisher.publish(linear_base_command)
        self.get_logger().info(f'Published linear base PWM: {pwm}')

def main(args=None):
    rclpy.init(args=args)

    linear_base_control = LinearBaseController()

    try:
        rclpy.spin(linear_base_control)
    except KeyboardInterrupt:
        print("Shutting down due to KeyboardInterrupt")
    except Exception as e:
        print(f"Exception {e}")
    finally:
        linear_base_control.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()
