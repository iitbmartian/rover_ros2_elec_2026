#!/usr/bin/env python3

import rclpy
from rclpy.node import Node
from std_msgs.msg import UInt16, Int8, String
from drive_control_interfaces.msg import DriveData, TelemetryData
import serial
import struct
import numpy as np

NUM_ENCODERS = 1
NUM_QUAD = 5 + 1  # 5 from timer, 1 through GPIO
NUM_ACS = 9


# UART SENDING FRAME:
# [11 pwm values 12 bit (2 byte with top MSB as direction) -> so 0000xxxx xxxxxxxxx and 1000xxxx xxxxxxxx are different directions, 12 last bits as pwm]
# [1 byte as command for wrist steppers]
# total 11*2 = 22 bytes + 1 = 23

class UARTBridge(Node):
    def __init__(self):
        super().__init__('uart_bridge')

        # UART object
        self.serial = serial.Serial('/dev/ttyUSB0', 115200, timeout=0.08, bytesize=serial.EIGHTBITS,
                                    parity=serial.PARITY_NONE, stopbits=serial.STOPBITS_ONE)

        # self.recv_buffer = bytearray()

        # Subscribe to nodes
        self.create_subscription(DriveData, '/drive_commands', self.drive_callback, 10)
        # self.create_subscription(LinearBaseData, '/linear_base_commands', self.linear_base_callback, 10)
        # self.create_subscription(WristData, '/wrist_commands', self.wrist_callback, 10)

        # Timer: to send UART frame -> 100 hz for 0.01 callback
        self.send_timer = self.create_timer(0.01, self.send_uart)

        # Timer: read incoming UART frame
        self.read_timer = self.create_timer(0.1, self.read_uart)

        # Telemetry Publisher
        self.telemetry_publisher = self.create_publisher(TelemetryData, '/telemetry', 10)

        self.motor_values = [0] * 2
        # self.wrist_command = 4  # do nothing
        self.serial.reset_input_buffer()

    def add_direction(self, pwm_vals, directions):
        result = []
        for pwm, direction in zip(pwm_vals, directions):
            # convert direction to 0 or 1 for MSB
            dir_bit = 1 if direction < 0 else 0  # set MSB = 1 for reverse

            # 12-bit PWM masked
            # pwm_12 = pwm & 0x0FFF
            pwm_12 = pwm & 0x00FF

            # final 16-bit value
            # combined = (dir_bit << 15) | pwm_12
            combined = (dir_bit << 8) | pwm_12

            result.append(combined)

        return result

    def drive_callback(self, msg):
        drive = self.add_direction(msg.pwm, msg.direction)
        self.motor_values = drive

    # def linear_base_callback(self, msg):
    #     linear = self.add_direction([msg.pwm], [msg.direction])
    #     self.motor_values[10] = linear[0]
    #
    # def wrist_callback(self, msg):
    #     if not msg.enable:
    #         self.wrist_command = 4
    #         return
    #
    #     if msg.direction == [1, 1]:
    #         self.wrist_command = 0
    #     elif msg.direction == [-1, -1]:
    #         self.wrist_command = 1
    #     elif msg.direction == [-1, 1]:
    #         self.wrist_command = 2
    #     else:
    #         self.wrist_command = 3

    def send_uart(self):
        fmt = '>2H'  # tells uart payload frame type, 11 H (unsigned 16 bit) + 1 B (unsigned 8 bit)
        print(self.motor_values)
        payload = struct.pack(fmt, *self.motor_values)
        print(len(payload), payload)
        self.serial.write(payload)

    def read_uart(self):

        telemetry_data = TelemetryData()

        total_len = (2 * NUM_ENCODERS) + (12 * NUM_QUAD) + (2 * NUM_ACS)

        bytes_to_read = self.serial.in_waiting

        if bytes_to_read == 0:
            return

        data = self.serial.read(bytes_to_read)
        last_nl = data.rfind(b'\n')

        if last_nl == -1:
            print("Newline character not found")
            print("Debug info:")
            print(bytes_to_read, data)

            self.serial.reset_input_buffer()
            return

        print(last_nl)

        data = data[last_nl - total_len:last_nl]  # take only that slice frame

        print("Raw frame: ", data)
        print("Length of frame: ", len(data))

        if len(data) != total_len:
            return  # incomplete frame

        print('Reading uart frame')

        i = 0

        position = []
        speed = []
        acceleration = []

        for _ in range(NUM_QUAD):
            position_values = data[i:i + 4]
            position_value = int.from_bytes(position_values, byteorder='big', signed=True)
            position.append(position_value)
            print(f"Position {_ + 1}: {position_value}")
            i += 12

        i = 4

        for _ in range(NUM_QUAD):
            speed_values = data[i:i + 4]
            speed_value = int.from_bytes(speed_values, byteorder='big', signed=True)
            speed.append(speed_value)
            print(f"Speed {_ + 1}: {speed_value}")
            i += 12

        i = 8

        for _ in range(NUM_QUAD):
            acceleration_values = data[i:i + 4]
            acceleration_value = int.from_bytes(acceleration_values, byteorder='big', signed=True)
            acceleration.append(acceleration_value)
            print(f"Acceleration {_ + 1}: {acceleration_value}")
            i += 12

        angles = []

        i = 12 * (NUM_QUAD)  # force define the offset

        for _ in range(NUM_ENCODERS):
            magnetic_values = data[i:i + 2]
            magnetic_value = int.from_bytes(magnetic_values, byteorder='big', signed=False)
            value = magnetic_value / 100
            print(f"Angle {_ + 1}: {value}")
            angles.append(value)
            i += 2

        current = []

        for _ in range(NUM_ACS):
            current_values = data[i:i + 2]
            current_value = int.from_bytes(current_values, byteorder='big', signed=False)
            value = current_value * 0.01208791208 - 25  # scaling factor to convert from ADC 0-4095 to current value
            print(f"Current {_ + 1}: {value}")
            current.append(value)
            i += 2

        telemetry_data.angles = angles
        telemetry_data.current = current

        telemetry_data.position = position
        telemetry_data.speed = speed
        telemetry_data.acceleration = acceleration

        print("Publishing telemetry data")
        self.serial.reset_input_buffer()

        self.telemetry_publisher.publish(telemetry_data)


def main(args=None):
    rclpy.init(args=args)
    node = UARTBridge()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()


if __name__ == "__main__":
    main()