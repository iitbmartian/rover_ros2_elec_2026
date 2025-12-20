import rclpy
from rclpy.node import Node

from sensor_msgs.msg import Joy
from all_interfaces.msg import ArmEndMotion, EncoderArm, TelemetryData

toggle = False
gripper_state = 255 

class JoystickControlNode(Node):
    def __init__(self):
        super().__init__('arm_joystick_commands')

        # Publisher for arm motion commands
        self.linear_base_pub = self.create_publisher(ArmEndMotion, 'arm_commands', 10)

        # Publisher for the 'joint_states' topic
        # self.publisher_ = self.create_publisher(JointState, 'joint_states', 10)

        # Subscriber for joystick inputs
        self.joystick_subscription = self.create_subscription(
            Joy, '/joy', self.joystick_callback, 10
        )
        self.receive_encoder = self.create_subscription(EncoderArm, 'encoder_arm', self.encoder_callback, 10)
        self.receive_base_encoder = self.create_subscription(TelemetryData, 'telemetry', self.base_encoder_callback, 10)

        # Initialize arm motion command message
        self.arm_command = ArmEndMotion()

        self.base_speed = 0
        self.base_direction = 1
        
        self.joint2_speed = 0
        self.joint2_direction = 1

        self.left_motor_speed = 0
        self.left_motor_direction = 1

        self.right_motor_speed = 0
        self.right_motor_direction = 1

        self.joint3_speed = 0
        self.joint3_direction = 1

        self.current_base_pos = 0
        self.current_shoulder_pos = 0
        self.current_elbow_pos = 0

        # robot.q[0] = 0.0
        # robot.q[1] = -0.57
        # robot.q[2] = -0.12

        self.min_base_pos = 2000
        self.max_base_pos = 115000

        self.max_shoulder_pos = 8600
        self.min_shoulder_pos = -8600

        self.max_elbow_pos = 4400
        self.min_elbow_pos = -9200

        # Initialize desired velocity and joint state
        # self.desired_velocity = np.zeros(6)  # [vx, vy, vz, wx, wy, wz]
        # self.joint_state = JointState()
        # self.joint_state.name = [f'joint_{i+1}' for i in range(len(robot.q))]
        # self.joint_state.position = list(robot.q)

    def base_encoder_callback(self, msg: TelemetryData):
        self.current_base_pos = msg.position[4]

    def encoder_callback(self, msg: EncoderArm):
        """
        Callback for receiving the current encoder pulse count.
        """
        # self.current_base_pos = msg.arm_node2[0] # min - 10698, home - 65536, max - 117678
        self.current_shoulder_pos = msg.arm_node0[0] # min - -8650, home - 0, max - 8680
        self.current_elbow_pos = msg.arm_node1[0] # min - -9361, home - 0, max - 4680 


         # Convert from continuous joint angles to desired encoder positions
        # robot.q[0] = -0.21 + ((0.45 * (self.current_base_pos)) / 117678) # motor_side - 0.24, home (middle) - 0.0, other_side - -0.21
        # robot.q[1] = -2.13 + ((3.13 * (self.current_shoulder_pos + 8650)) / 17330) # home (90 deg) - -0.57, min (0 deg) - -2.13, max (180 deg) - 1.0 
        # robot.q[2] = -3.27 + ((4.71 * (self.current_elbow_pos + 9361)) / 14041) # home (90 deg) -0.12, min (0 deg) - -3.27, max (180 deg) - 1.44
        # robot.q[3] = 0
        # robot.q[4] = 0


    def joystick_callback(self, joystick):
        global toggle
        global gripper_state

        a_butt = joystick.buttons[0]
        b_butt = joystick.buttons[1]
        x_butt = joystick.buttons[2]
        y_butt = joystick.buttons[3]

        lt_butt = joystick.axes[2]
        rt_butt = joystick.axes[5]

        cross_ver = joystick.axes[7]

        right_hor = joystick.axes[3]
        right_ver = joystick.axes[4]

        start_button = joystick.buttons[7]
        back_button = joystick.buttons[6]

        rb_butt=joystick.buttons[5]


        if rb_butt == 1:
            self.arm_command.reset = True
        else:
            self.arm_command.reset = False

        if x_butt == 1:
            self.left_motor_speed = 120
            self.left_motor_direction = 1

            self.right_motor_speed = 120
            self.right_motor_direction = 0


        elif b_butt == 1:
            self.left_motor_speed = 120
            self.left_motor_direction = 0

            self.right_motor_speed = 120
            self.right_motor_direction = 1

        elif y_butt == 1:
            self.left_motor_speed = 120
            self.left_motor_direction = 1

            self.right_motor_speed = 120
            self.right_motor_direction = 1

        elif a_butt == 1:
            self.left_motor_speed = 120
            self.left_motor_direction = 0

            self.right_motor_speed = 120
            self.right_motor_direction = 0  
        
        else:
            self.left_motor_direction = 0
            self.left_motor_speed = 0
            self.right_motor_speed = 0
            self.right_motor_direction = 0

        if lt_butt < 1.0:
            if lt_butt >= 0.9:
                self.joint3_speed = 0
                self.joint3_direction = 0
            else: 
                self.joint3_speed = int(((0.9 - lt_butt) / 1.9)*200)
                self.joint3_direction = 0

        elif rt_butt < 1.0:
            if rt_butt >= 0.9:
                self.joint3_speed = 0
                self.joint3_direction = 1
            else: 
                self.joint3_speed = int(((0.9 - rt_butt) / 1.9)*200)
                self.joint3_direction = 1
        
        else:
            self.joint3_direction = 0
            self.joint3_speed = 0


        if right_ver < 0:  
            if abs(right_ver) <= 0.1:
                self.joint2_speed = 0
                self.joint2_direction = 0
            else: 
                self.joint2_speed = int(((abs(right_ver) - 0.1) / 0.9)*255)
                self.joint2_direction = 0

        elif right_ver > 0:  
            if right_ver <= 0.1:
                self.joint2_speed = 0
                self.joint2_direction = 1

            else:
                self.joint2_speed = int(((right_ver - 0.1) / 0.9)*255) 
                self.joint2_direction = 1  
        
        else:
            self.joint2_direction = 0
            self.joint2_speed = 0

        # elif right_hor < 0:  
        #     if self.current_base_position >= 5000 and self.current_base_position <= 114200:
        #         if abs(right_hor) <= 0.1:
        #             self.base_speed = 0
        #             self.base_direction = 1
        #         else: 
        #             self.base_speed = int(((abs(right_hor) - 0.1) / 0.9)*255)
        #             self.base_direction = 1
        #     else:
        #             self.base_speed = 0
        #             self.base_direction = 1


        # elif right_hor > 0:  
        #     if self.current_base_position >= 5000 and self.current_base_position <= 114200:
        #         if right_hor <= 0.1:
        #             self.base_speed = 0
        #             self.base_direction = 0

        #         else:
        #             self.base_speed = int(((right_hor - 0.1) / 0.9)*255) 
        #             self.base_direction = 0  
        #     else:
        #             self.base_speed = 0
        #             self.base_direction = 0

        if right_hor < 0:  
            if abs(right_hor) <= 0.1:
                self.base_speed = 0
                self.base_direction = 1
            else: 
                self.base_speed = int(((abs(right_hor) - 0.1) / 0.9)*4095)
                self.base_direction = 1


        elif right_hor > 0:  
            if right_hor <= 0.1:
                self.base_speed = 0
                self.base_direction = 0

            else:
                self.base_speed = int(((right_hor - 0.1) / 0.9)*4095) 
                self.base_direction = 0 
        else:
            self.base_direction = 0
            self.base_speed = 0 


        # else:
        #     self.base_speed = 0
        #     self.base_direction = 1

        #     self.joint2_speed = 0
        #     self.joint2_direction = 1

        #     self.left_motor_speed = 0
        #     self.left_motor_direction = 1

        #     self.right_motor_speed = 0
        #     self.right_motor_direction = 1

        #     self.joint3_speed = 0
        #     self.joint3_direction = 1   

        if start_button:
            toggle = True
        if back_button:
            toggle = False

        # # Enforce joint limits
        # joint_limits = [
        #     (-0.38, 0.40),   # Joint 1 limits
        #     (-1.5708, 1.5708),  # Joint 2 limits
        #     (-4.0, 3.1416),  # Joint 3 limits
        #     (-1.5708, 1.5708),  # Joint 4 limits
        #     # Add more limits as needed for additional joints
        # ]

        #for i, (lower, upper) in enumerate(joint_limits):
        #    robot.q[i] = np.clip(robot.q[i], lower, upper)  # Constrain joint position

        #self.joint_state.position = [round(q, 2) for q in robot.q]

        # Set the header timestamp to current time
        #self.joint_state.header.stamp = self.get_clock().now().to_msg()
        
        # Publish the joint states
        #self.publisher_.publish(self.joint_state)

        # --------------------
        #  Example after computing pwm for base
        # --------------------
        if (self.current_base_pos <= self.min_base_pos and self.base_direction == 1):
            # Already at or below the minimum, and user wants to move further down
            self.base_speed = 0
        elif (self.current_base_pos >= self.max_base_pos and self.base_direction == 0):
            # Already at or above the maximum, and user wants to move further up
            self.base_speed = 0

        # Similarly for joint2
        # if (self.current_shoulder_pos >= self.max_shoulder_pos and self.joint2_direction == 1):
        #     self.joint2_speed = 0
        # elif (self.current_shoulder_pos <= self.min_shoulder_pos and self.joint2_direction == 0):
        #     self.joint2_speed = 0

        # Similarly for joint3
        # if (self.current_elbow_pos >= self.max_elbow_pos and self.joint3_direction == 1):
        #     self.joint3_speed = 0
        # elif (self.current_elbow_pos <= self.min_elbow_pos and self.joint3_direction == 0):
        #     self.joint3_speed = 0

        self.arm_command.sys_check = toggle
        self.arm_command.speed = [self.joint2_speed, self.joint3_speed, self.base_speed, self.left_motor_speed, self.right_motor_speed, 0] # last element placeholder for gripper
        self.arm_command.direction = [self.joint2_direction, self.joint3_direction, self.base_direction, self.left_motor_direction, self.right_motor_direction, 0] # last element placeholder for gripper

        # Debug information
        self.get_logger().info(f"Sending command to hardware: PWM {self.arm_command.speed}, DIR {self.arm_command.direction}, Syscheck {self.arm_command.sys_check}, Reset {self.arm_command.reset}") 

        self.arm_publisher.publish(self.arm_command)


def main(args=None):
    rclpy.init(args=args)
    node = JoystickControlNode()

    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()
                   

'''
pin=18
freq=50

#Setting PWM according to the Servo motor chosen
h = lgpio.gpiochip_open(0)

#Turning the rover by the angle necessary
def Turn(angle_to_point):
    duty = ((angle_to_point/90)*5)+7.5
    print(duty)
    if duty<2.5:
        duty=2.5
    lgpio.tx_pwm(h, pin, freq, duty)
    sleep(0.1)
    return

class GripperControlNode(Node):
    def __init__(self):
        super().__init__('gripper_testing')

        #  Publisher for arm motion commands
        self.gripper_cmd_subscriber = self.create_subscription(ArmEndMotion, 'arm_commands', self.jo>

        # Initialize arm motion command message
        self.received_arm_command = ArmEndMotion()

    def joystick_callback(self, cmd):
        #get the last message of arm commands
        val = cmd.speed[-1]

        if val == 0:
            Turn(-90)
        if val == 255:
            Turn(45)


def main(args=None):
    rclpy.init(args=args)
    node = GripperControlNode()

    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()
'''

