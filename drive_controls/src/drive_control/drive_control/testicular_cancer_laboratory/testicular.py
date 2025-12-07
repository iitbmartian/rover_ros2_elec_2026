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


    def get_uart_data(self):
        bytesToRead = self.ser.inWaiting()
        quad_data = []
        print(bytesToRead)
        data = self.ser.read(bytesToRead)
        print(data)
        # last_nl = data.rfind(b'\n')
        # last24 = data[last_nl - 24:last_nl] if last_nl != -1 else data[-24 - 1:-1]
        # for i in range(4):
        #     base = 12 * i
        #     pos = int.from_bytes(last24[base:base + 4], byteorder='little', signed=True)
        #     diff = int.from_bytes(last24[base + 4:base + 8], byteorder='little', signed=True)
        #     diff2 = int.from_bytes(last24[base + 8:base + 12], byteorder='little', signed=True)
        #     quad_data.append(pos)
        #
        # self.ser.reset_input_buffer()
        # print(quad_data)
        # self.pos_data = quad_data
        # return quad_data

        return data


tester = TestController()

crop = b""

def blop():
    global crop
    t1 = time.perf_counter()
    while time.perf_counter() - t1 < 0.1:
        crop += tester.get_uart_data()
        time.sleep(0.01)

try:
    tester.ser.read(tester.ser.in_waiting)

    powm = 0
    while True:
        tester.send_uart_data([0] * 2, [powm, 0] * 2)
        blop()
        powm += 1

        with open("testicle_readings.txt", 'w') as f:
            f.write(crop.hex())
except:
    tester.send_uart_data([0] * 2, [0] * 2)
    tester.ser.close()
    

