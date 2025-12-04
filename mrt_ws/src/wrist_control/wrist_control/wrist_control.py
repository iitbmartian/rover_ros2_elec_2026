#!/usr/bin/env python

import rclpy
from rclpy.node import Node
from sensor_msgs.msg import Joy
from all_interfaces.msg import WristData

class WristController(Node):
    def __init__(self):
        super().__init__('wrist_controller')

        # Create a subscription to Joystick 
        self.joystick_subscription = self.create_subscription(Joy,'/joy',self.joystick_callback,10)

        # Create a publisher for Wrist Commands
        self.wrist_publisher = self.create_publisher(WristData,'/wrist_commands',10)
        
    def joystick_callback(self,joy_val):
        wrist_commands = WristData()

        # Extract Joystick Values (These are either 0 or 1)
        a = joy_val.buttons[0]
        b = joy_val.buttons[1]
        x = joy_val.buttons[2]
        y = joy_val.buttons[3]

        wrist_commands.pulse_interval = [800,800]

        wrist_commands.enable = True

        if(y == 1):
            wrist_commands.direction = [1,1]
        elif(a == 1):
            wrist_commands.direction = [-1,-1]
        elif(x == 1):
            wrist_commands.direction = [-1,1]
        elif(b == 1):
            wrist_commands.direction = [1,-1]
        else:
            wrist_commands.direction = [1,1]
            wrist_commands.enable = False

def main(args=None):
    rclpy.init(args=args)

    drive_control = WristController()

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
