#!/usr/bin/env python3

import rclpy
from rclpy.node import Node
from std_msgs.msg import UInt16, Int8, String
from all_interfaces.msg import DriveData, LinearBaseData, WristData, TelemetryData
import serial
import struct

#UART SENDING FRAME:
#[11 pwm values 12 bit (2 byte with top MSB as direction) -> so 0000xxxx xxxxxxxxx and 1000xxxx xxxxxxxx are different directions, 12 last bits as pwm]
#[1 byte as command for wrist steppers]
# total 11*2 = 22 bytes + 1 = 23

class UARTBridge(Node):
    def __init__(self):
        super().__init__('uart_bridge')

        # UART object
        self.serial = serial.Serial('/dev/ttyUSB0', 115200, timeout=0.001)

        # Subscribe to nodes
        self.create_subscription(DriveData,'/drive_commands',self.drive_callback,10)
        self.create_subscription(LinearBaseData,'/linear_base_commands',self.linear_base_callback,10)
        self.create_subscription(WristData,'/wrist_commands',self.wrist_callback,10)


        # Timer: to send UART frame -> 10 khz for 0.001 callback
        self.send_timer = self.create_timer(0.001, self.send_uart)
        
        # Telemetry Publisher
        self.telemetry_publisher = self.create_publisher(TelemetryData, '/telemetry', 10)

        self.motor_values = [0]*11
        self.wrist_command = 4 #do nothing 
    
    def add_direction(self,pwm_vals,directions):
        result = []
        for pwm, direction in zip(pwm_vals, directions):

            # convert direction to 0 or 1 for MSB
            dir_bit = 1 if direction < 0 else 0  # set MSB = 1 for reverse

            # 12-bit PWM masked
            pwm_12 = pwm & 0x0FFF

            # final 16-bit value
            combined = (dir_bit << 15) | pwm_12

            result.append(combined)

        return result
    
    def drive_callback(self,msg):
        drive = self.add_direction(msg.pwm,msg.direction)
        self.motor_values[:10] = drive

    def linear_base_callback(self,msg):
        linear = self.add_direction([msg.pwm],[msg.direction])
        self.motor_values[10] = linear[0]
    
    def wrist_callback(self,msg):
        if not msg.enable:
            self.wrist_command = 4
            return

        if msg.direction == [1, 1]:
            self.wrist_command = 0
        elif msg.direction == [-1, -1]:
            self.wrist_command = 1
        elif msg.direction == [-1, 1]:
            self.wrist_command = 2
        else:
            self.wrist_command = 3

    def send_uart(self):
        fmt = '>11HB' #tells uart payload frame type, 11 H (unsigned 16 bit) + 1 B (unsigned 8 bit)
        payload = struct.pack(fmt,*self.motor_values,self.wrist_command)
        self.serial.write(payload)

def main(args=None):
    rclpy.init(args=args)
    node = UARTBridge()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()


if __name__ == "__main__":
    main()