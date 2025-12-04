#!/usr/bin/env python

import rclpy
from rclpy.node import Node
from sensor_msgs.msg import Joy 
import requests
import math 

class Gripper(Node):
    def __init__(self):
        super().__init__('gripper')
        
        self.esp32_ip = '192.168.69.20'
        self.esp32_port = 67
        self.server_url = f'http://{self.esp32_ip}:{self.esp32_port}/'
        
        self.joystick_subscription = self.create_subscription(Joy,'/joy',self.joystick_callback,10)
        
    def joystick_callback(self, joy_val):
        
        #Has to be changed depending on what joystick button is chosen at the end
        raw_axis_value = joy_val.axes[4] 
        
        motor_speed = int((raw_axis_value + 1.0) * 50.0)
        
        motor_speed = max(0, min(100, motor_speed))

        self.send_esp32_command(motor_speed)
            
            
    def send_esp32_command(self, new_number):
        try:
            # Construct the URL
            command_url = f'{self.server_url}?{new_number}'
            
            # Use requests.get to send the command
            response = requests.get(command_url, timeout=0.5)
            response.raise_for_status() # Check for HTTP errors (4xx or 5xx)
                        
        except requests.exceptions.RequestException as e:
            # Log connection/timeout errors (throttled to avoid spamming the console)
            self.get_logger().warn(f"Failed to command ESP32 at {self.server_url}: {e}", throttle_duration_sec=5.0)
        except Exception as e:
            self.get_logger().error(f"An unexpected error occurred: {e}")
    
    #Add a way to get the camera feed back from esp-cam
    
def main(args=None):
    rclpy.init(args=args)
    node = Gripper()
    
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        node.get_logger().info("Shutting down due to KeyboardInterrupt")
    finally:
        node.destroy_node()
        rclpy.shutdown()

if __name__ == '__main__':
    main()