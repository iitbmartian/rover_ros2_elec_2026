# !/usr/bin/env python
import time
import serial


def f1():
    NUM_QUADS = 5
    NUM_ENCODERS = 1
    NUM_ASC = 9

    FRAME = {}
    for i in range(1, NUM_QUADS + 1):
        FRAME[f"QUAD {i} POS"] = 4
        FRAME[f"QUAD {i} DIFF 1"] = 4
        FRAME[f"QUAD {i} DIFF 2"] = 4

    FRAME["DRILL QUAD POS"] = 4
    FRAME["DRILL QUAD DIFF 1"] = 4
    FRAME["DRILL QUAD DIFF 2"] = 4

    for i in range(1, NUM_ENCODERS + 1):
        FRAME[f"MAG {i} POS"] = 2

    for i in range(1, NUM_ASC + 1):
        FRAME[f"ASC {i} VAL"] = 2

    FRAME["NEWLINE"] = 1
    return FRAME


ts = []
poses = []
diffs = []


def display(data, frame=f1()):
    max_length = max(map(lambda x: len(x), frame.keys()))
    for name, bits in frame.items():
        bop = data[:bits]
        data = data[bits:]
        val = int.from_bytes(bop, byteorder='big', signed=True)
        print(f"{name.ljust(max_length, ' ')}: {val}")

        if name == "QUAD 2 POS":
            ts.append(time.perf_counter())
            poses.append(val)
        if name == "QUAD 2 DIFF 1":
            diffs.append(val)
    print("-" * 67)


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

        self.uart_frame = f1()

    def send_uart_data(self, dir_arr, pwm_arr):
        print(dir_arr)
        outstring = b""
        for i in range(len(dir_arr)):
            outstring += dir_arr[i].to_bytes(1, byteorder='big')
            outstring += min(int(pwm_arr[i]), 255).to_bytes(1, byteorder='big')
        # add current and target height for the z axis stepperrint(f'Sending serial: {outstring}')
        self.ser.write(outstring)

    def get_uart_data(self):
        bytesToRead = self.ser.inWaiting()
        quad_data = []
        # print(bytesToRead)
        if bytesToRead >= sum(self.uart_frame.values()):
            data = self.ser.read(sum(self.uart_frame.values()))
            print(data)

            return data

        return None


tester = TestController()

vels = []


def blop():
    t1 = time.perf_counter()
    while time.perf_counter() - t1 < 1:
        crop = tester.get_uart_data()
        if crop is not None:
            display(crop, f1())

        time.sleep(0.005)


try:
    tester.ser.read(tester.ser.in_waiting)
    while True:
        tester.send_uart_data([0, 0], [50, 50])
        blop()
        tester.send_uart_data([1, 1], [50, 50])
        blop()
except BaseException as e:
    print("DIE", e)
    tester.send_uart_data([0] * 2, [0] * 2)
    tester.ser.read(tester.ser.in_waiting)
    tester.ser.close()

from matplotlib import pyplot as plt

vels = []
v_time = 0.1

for i in range(len(ts)):
    t = ts[i]
    p = poses[i]

    other_t = t
    other_p = p
    for other_t, other_p in list(zip(ts, poses))[i - 1::-1]:
        if (t - other_t) > v_time:
            break
    if t == other_t:
        gom = 0
    else:
        gom = (p - other_p) / (t - other_t)

    vels.append(gom)

# plt.plot(ts, vels)
# plt.plot(ts, diffs)
# plt.show()

fig, ax1 = plt.subplots()

color = 'tab:red'
ax1.set_xlabel('time (s)')
ax1.set_ylabel('vels', color=color)
ax1.plot(ts, vels, color=color)
ax1.tick_params(axis='y', labelcolor=color)

ax2 = ax1.twinx()  # instantiate a second Axes that shares the same x-axis

color = 'tab:blue'
ax2.set_ylabel('diffs', color=color)  # we already handled the x-label with ax1
ax2.plot(ts, diffs, color=color)
ax2.tick_params(axis='y', labelcolor=color)

ax3 = ax1.twinx()  # instantiate a second Axes that shares the same x-axis

color = 'tab:green'
ax3.set_ylabel('poses', color=color)  # we already handled the x-label with ax1
ax3.plot(ts, poses, color=color)
ax3.tick_params(axis='y', labelcolor=color)



fig.tight_layout()  # otherwise the right y-label is slightly clipped
plt.show()
