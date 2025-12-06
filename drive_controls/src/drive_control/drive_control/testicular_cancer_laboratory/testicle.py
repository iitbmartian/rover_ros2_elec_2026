#!/usr/bin/env python
import time
import serial


class TestController:

    def __init__(self):
        self.ser = serial.Serial(
            port='/dev/ttyUSB0',  # Update this to your serial port
            baudrate=115200,
            parity=serial.PARITY_NONE,
            stopbits=serial.STOPBITS_ONE,
            bytesize=serial.EIGHTBITS,
            timeout=100  # Timeout in seconds for read operations
        )
        try:
            self.ser.close()
        except:
            pass
        self.ser.open()

    def send_uart_data(self, dir_arr, pwm_arr):
        print(dir_arr)
        outstring = b""
        for i in range(len(dir_arr)):
            outstring += dir_arr[i].to_bytes(1, byteorder='little')
            outstring += min(int(pwm_arr[i]), 255).to_bytes(1, byteorder='little')
        # add current and target height for the z axis stepperrint(f'Sending serial: {outstring}')
        self.ser.write(outstring)


tester = TestController()

while True:
    tester.send_uart_data([1] * 2, [128] * 2)
    time.sleep(1)
    tester.send_uart_data([0] * 2, [128] * 2)
    time.sleep(1)

