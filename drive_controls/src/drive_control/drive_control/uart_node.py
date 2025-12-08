#!/usr/bin/env python3

import rclpy
from rclpy.node import Node
from std_msgs.msg import UInt16, Int8, String
from drive_control_interfaces.msg import DriveData, DriveEncoderData
import serial
import struct

NUM_ENCODERS = 1
NUM_QUAD = 5  # 5 from timer, 1 through GPIO
NUM_ACS = 9


# UART SENDING FRAME:
# [11 pwm values 12 bit (2 byte with top MSB as direction) -> so 0000xxxx xxxxxxxxx and 1000xxxx xxxxxxxx are different directions, 12 last bits as pwm]
# [1 byte as command for wrist steppers]
# total 11*2 = 22 bytes + 1 = 23

class UARTBridge(Node):
    def __init__(self):
        super().__init__('uart_bridge')

        # UART object
        self.serial = serial.Serial('/dev/ttyUSB0', 115200, timeout=0.001)

        # Subscribe to nodes
        self.create_subscription(DriveData, '/drive_commands', self.drive_callback, 10)

        self.encoder_publisher = self.create_publisher(DriveEncoderData, '/drive_encoders', 10)

        # Timer: to send UART frame -> 100 Hz for 0.01 callback
        self.send_timer = self.create_timer(0.01, self.send_uart)

        # Timer: read incoming UART frame
        self.read_timer = self.create_timer(0.01, self.read_uart)


        self.motor_values = [0] * 16
        self.wrist_command = 4  # do nothing

    def drive_callback(self, msg):
        for i in range(8):
            self.motor_values[2 * i] = msg.dir_arr[i]
            self.motor_values[2 * i + 1] = msg.pwm_arr[i]

    def send_uart(self):
        # print(self.motor_values)
        outstring = b""
        for bye in self.motor_values:
            outstring += bye.to_bytes(1, byteorder='big')

        self.serial.write(outstring)

    def get_frame(self):
        FRAME = {}
        for i in range(1, NUM_QUAD + 1):
            FRAME[f"QUAD {i} POS"] = 4
            FRAME[f"QUAD {i} DIFF 1"] = 4
            FRAME[f"QUAD {i} DIFF 2"] = 4

        FRAME["DRILL QUAD POS"] = 4
        FRAME["DRILL QUAD DIFF 1"] = 4
        FRAME["DRILL QUAD DIFF 2"] = 4

        for i in range(1, NUM_ENCODERS + 1):
            FRAME[f"MAG {i} POS"] = 2

        for i in range(1, NUM_ACS + 1):
            FRAME[f"ASC {i} VAL"] = 2

        FRAME["NEWLINE"] = 1

        return FRAME

    def read_uart(self):
        # print("readdingg")

        total_len = (2 * NUM_ENCODERS) + (12 * (NUM_QUAD + 1)) + (2 * NUM_ACS) + 1

        try:
            # print(self.serial.in_waiting)
            data = self.serial.read(total_len)

            if len(data) != total_len:
                return  # incomplete frame

            frame = self.get_frame()

            magnetics = []

            max_length = max(map(lambda x: len(x), frame.keys()))
            for name, bits in frame.items():
                bop = data[:bits]
                data = data[bits:]
                val = int.from_bytes(bop, byteorder='big', signed=True)
                print(f"{name.ljust(max_length, ' ')}: {val}")

                if name == "MAG 1 POS":
                    magnetics.append(val / 100)
            print("-" * 67)


            encoa_data = DriveEncoderData()
            encoa_data.magnetic_data = magnetics

            self.encoder_publisher.publish(encoa_data)
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