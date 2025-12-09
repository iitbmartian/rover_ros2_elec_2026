#!/usr/bin/env python3

import rclpy
from rclpy.node import Node
from std_msgs.msg import UInt16, Int8, String
from all_interfaces.msg import DriveData, LinearBaseData, WristData, TelemetryData
from rclpy.qos import QoSProfile, ReliabilityPolicy
import serial
import struct

NUM_ENCODERS = 6
NUM_QUAD = 5 + 1 # 5 from timer, 1 through GPIO
NUM_ACS = 11

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


        # Timer: to send UART frame -> 100 hz for 0.01 callback
        #self.send_timer = self.create_timer(0.01, self.send_uart)

        # Timer: read incoming UART frame
        self.read_timer = self.create_timer(0.01, self.read_uart)
        
        # Telemetry Publisher
        self.telemetry_publisher = self.create_publisher(TelemetryData, '/telemetry',QoSProfile(depth=10, reliability=ReliabilityPolicy.BEST_EFFORT))

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

    def read_uart(self):

        telemetry_data = TelemetryData()

        total_len = (2 * NUM_ENCODERS) + (12 * NUM_QUAD) + (2 * NUM_ACS)
        try:
            data = self.serial.read(total_len)

            if len(data) != total_len:
                return   # incomplete frame
            
            print('Reading uart frame')
            i = 0
            

            position = []
            speed = []
            acceleration = []

            for _ in range(NUM_QUAD):
                position_values = data[i:i+4] 
                position_value = (position_values[0] | position_values[1] << 8 | position_values[2] << 16 | position_values[3] << 24)
                position.append(position_value)
                i += 4
            
            for _ in range(NUM_QUAD):
                speed_values = data[i:i+4] 
                speed_value = (speed_values[0] | speed_values[1] << 8 | speed_values[2] << 16 | speed_values[3] << 24)
                speed.append(speed_value)
                i += 4
            
            for _ in range(NUM_QUAD):
                acceleration_values = data[i:i+4]
                acceleration_value = (acceleration_values[0] | acceleration_values[1] << 8 | acceleration_values[2] << 16 | acceleration_values[3] << 24)
                acceleration.append(acceleration_value)
                i += 4

            telemetry_data.position = position
            telemetry_data.speed = speed
            telemetry_data.acceleration = acceleration
            
            angle = []

            for _ in range(NUM_ENCODERS):
                high_byte = data[i]
                low_byte = data[i+1]
                value = float((high_byte << 8) | low_byte)/100
                angle.append(value)
                i += 2

            telemetry_data.angle = angle
            
            current = []

            for _ in range(NUM_ACS):
                high_byte = data[i]
                low_byte = data[i+1]
                value = float((high_byte << 8) | low_byte)*0.01208791208 - 25 #scaling factor to convert from ADC 0-4095 to current value
                current.append(value)
                i += 2
            
            telemetry_data.current = current

            print("Publishing telemetry data")

            self.telemetry_publisher.publish(telemetry_data)


        except Exception:
            pass

def main(args=None):
    rclpy.init(args=args)
    node = UARTBridge()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()


if __name__ == "__main__":
    main()