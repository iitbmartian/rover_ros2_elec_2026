import serial
import time
import signal
import sys
import numpy as np
import rclpy
from rclpy.node import Node
from geometry_msgs.msg import Twist
from std_msgs.msg import Float64



class MotorController(Node):
    def __init__(self):
        super().__init__('uart_motor_controller')

        # global variables to use in PID section
        self.dir_left = 0
        self.dir_right = 0
        self.left_enc_velocity, self.right_enc_velocity = 0, 0
        self.curr_height, self.target_height = 0.0, 0.0

        self.angle_arr = [0, 0, 0, 0]
        self.vel_arr = [0, 0, 0, 0]

        self.z_radius = 0.01  # Radius of the lift mechanism wheel in meters
        self.z_scaling = 100.0 / (self.z_radius * np.pi)  # Scaling factor to convert height to encoder counts

        self.accepted_error = 5

        self.integral_l, self.integral_r = 0.0, 0.0
        self.integral_limit = 5000.0  # anti-windup clamp for integrator
        self.last_time = time.time()
        self.error_l, self.error_r = 0.0, 0.0
        self.last_error_l, self.last_error_r = 0.0, 0.0
        self.last_derivative_l, self.last_derivative_r = 0.0, 0.0
        self.derivative_l, self.derivative_r = 0.0, 0.0
        self.diff2_l, self.diff2_r = 0.0, 0.0
        self.pwmout_left, self.pwmout_right = 0, 0
        self.enc_data = [(0, 0, 0), (0, 0, 0)]

        self.dpwm_left, self.dpwm_right = 0.0, 0.0

        self.encs_per_rev = 3695  # Example value, set according to your encoder specifications
        self.dist_btw_wheels = 0.30  # Distance between wheels in meters
        self.radius_wheel = 0.05  # Radius of the wheels in meters

        self.polling_period = 0.2

        self.n_motors = 2
        self.kp = 0.1
        self.kd = 0.0
        self.ki = 0  # integral gain
    
        self.subscription = self.create_subscription(
            Twist,
            '/drive_commands',
            self.cmd_callback,
            10)
    
        self.ser = serial.Serial(
            port='/dev/ttyUSB0',  # Update this to your serial port
            baudrate=115200,
            parity=serial.PARITY_NONE,
            stopbits=serial.STOPBITS_ONE,
            bytesize=serial.EIGHTBITS,
            timeout=100  # Timeout in seconds for read operations
        )
        # Timeout parameters
        self.last_cmd_time = self.get_clock().now()
        self.timeout_duration = 0.5  # Stop if no message received for 0.5 seconds
        # Create timer to check for timeout
        self.timer = self.create_timer(self.polling_period, self.check_timeout)
        try:
            self.ser.close()
        except:
            pass
        self.ser.open()


    def send_uart_data(self, dir_arr, pwm_arr):
        outstring = b""
        for i in range(self.self.n_motors):
            outstring += dir_arr[i].to_bytes(1, byteorder='little')
            outstring += min(int(pwm_arr[i]), 255).to_bytes(1, byteorder='little')
        # add current and target height for the z axis stepperrint(f'Sending serial: {outstring}')
        self.ser.write(outstring)
    
    
    def get_uart_data(self):
        bytesToRead = self.ser.inWaiting()
        quad_data = []
        print(bytesToRead)
        data = self.ser.read(bytesToRead)
        last_nl = data.rfind(b'\n')
        last24 = data[last_nl - 24:last_nl] if last_nl != -1 else data[-24 - 1:-1]
        for i in range(2):
            base = 12 * i
            pos = int.from_bytes(last24[base:base + 4], byteorder='little', signed=True)
            diff = int.from_bytes(last24[base + 4:base + 8], byteorder='little', signed=True)
            diff2 = int.from_bytes(last24[base + 8:base + 12], byteorder='little', signed=True)
            quad_data.append((np.int32(pos), np.int32(diff), np.int32(diff2)))
    
        self.ser.reset_input_buffer()
        print(quad_data)
        return quad_data
    
    
    def cmd_callback(self, msg):
        # Update last command time
        self.last_cmd_time = self.get_clock().now()
        left_velocity = (msg.linear.x - (msg.angular.z * self.dist_btw_wheels / 2)) / self.radius_wheel
        right_velocity = (msg.linear.x + (msg.angular.z * self.dist_btw_wheels / 2)) / self.radius_wheel
        self.left_enc_velocity = left_velocity * self.encs_per_rev / (2 * np.pi) * self.polling_period
        self.right_enc_velocity = right_velocity * self.encs_per_rev / (2 * np.pi) * self.polling_period
        print(f"Setpoints: Left Enc Vel: {self.left_enc_velocity:.2f}, Right Enc Vel: {self.right_enc_velocity:.2f}")
        print(f"Encoder velocity: Left: {self.enc_data[0][1]}, Right: {self.enc_data[1][1]}")
        # print(f"Errors: Left: {self.error_l}, Right: {self.error_r}")
        # print(f"PWMs: Left: {self.pwmout_left}, Right: {self.pwmout_right}")
        # print(f"dPWM: Left: {self.dpwm_left}, Right: {self.dpwm_right}")
        if self.left_enc_velocity >= 0:
            self.dir_left = 1
        else:
            self.dir_left = 0
    
        if self.right_enc_velocity >= 0:
            self.dir_right = 1
        else:
            self.dir_right = 0
    
    
    def check_timeout(self):
    
        # Check if timeout has occurred
        current_time = self.get_clock().now()
        time_since_last_cmd = (current_time - self.last_cmd_time).nanoseconds / 1e9
    
        if time_since_last_cmd > self.timeout_duration:
            # Stop motors
            # for i in range(5):
            #       time.sleep(0.01)
            #       send_uart_data([0, 0], [0, 0])
            self.pwmout_left = 0
            self.pwmout_right = 0
            self.integral_l = 0.0
            self.integral_r = 0.0
            self.error_l = 0.0
            self.error_r = 0.0
            self.last_error_l = 0.0
            self.last_error_r = 0.0
            self.last_derivative_l = 0.0
            self.last_derivative_r = 0.0
    
            # Clear setpoints so PID doesn't immediately drive again
            self.left_enc_velocity = 0.0
            self.right_enc_velocity = 0.0
    
            self.dir_left = 0
            self.dir_right = 0
            self.get_logger().info('No cmd_vel received, stopping motors')
        try:
            if self.ser.isOpen():
    
                self.enc_data = self.get_uart_data()
                now = time.time()
                dt = now - self.last_time if now > self.last_time else 0.0
                self.last_time = now
    
                # first element of quad_data is right motor, second is left motor
                self.error_l = self.left_enc_velocity - float(self.enc_data[0][1])
                self.error_r = self.right_enc_velocity - float(self.enc_data[1][1])
    
                print(f"Left Error: {self.error_l}, Right Error: {self.error_r}")
                # integrate (clamp to avoid windup)
                self.integral_l += self.error_l * dt
                if self.integral_l > self.integral_limit:
                    self.integral_l = self.integral_limit
                elif self.integral_l < -self.integral_limit:
                    self.integral_l = -self.integral_limit
                self.integral_r += self.error_r * dt
                if self.integral_r > self.integral_limit:
                    self.integral_r = self.integral_limit
                elif self.integral_r < -self.integral_limit:
                    self.integral_r = -self.integral_limit
    
                self.derivative_l = (self.error_l - self.last_error_l) / dt if dt > 0.0 else 0.0
                self.diff2_l = (self.derivative_l - self.last_derivative_l) / dt if dt > 0.0 else 0.0
                self.last_error_l = self.error_l
                self.last_derivative_l = self.derivative_l
    
                self.derivative_r = (self.error_r - self.last_error_r) / dt if dt > 0.0 else 0.0
                self.diff2_r = (self.derivative_r - self.last_derivative_r) / dt if dt > 0.0 else 0.0
                self.last_error_r = self.error_r
                self.last_derivative_r = self.derivative_r
    
                # PID output
                self.dpwm_left = self.kp * self.error_l + self.ki * self.integral_l - self.kd * self.diff2_l
                self.dpwm_right = self.kp * self.error_r + self.ki * self.integral_r - self.kd * self.diff2_r
                # if abs(self.error_l) < self.accepted_error:
                #       self.dpwm_left = 0.0
                # if abs(self.error_r) < self.accepted_error:
                #       self.dpwm_right = 0.0
                self.pwmout_left += self.dpwm_left
                self.pwmout_right += self.dpwm_right
    
                print(f"Left PWM: {self.pwmout_left}, Right PWM: {self.pwmout_right}")
    
                self.dir_left = int(((self.pwmout_left < 0)))
                self.dir_right = int(((self.pwmout_right < 0)))
                self.send_uart_data([self.dir_left, self.dir_right],
                                    [min(abs(int(self.pwmout_left)), 255), min(abs(int(self.pwmout_right)), 255)])
        except serial.SerialException as e:
            print(f"Serial port error: {e}")
    
    
    def end_transmission(self):
        if self.ser.isOpen():
            try:
                self.send_uart_data([0, 0], [0, 0])
            except:
                pass
            self.ser.close()
    
    
def main(args=None):
    rclpy.init(args=args)
    motor_controller = MotorController()
    try:
        rclpy.spin(motor_controller)
    except KeyboardInterrupt:
        pass
    finally:
        motor_controller.end_transmission()
        motor_controller.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()
