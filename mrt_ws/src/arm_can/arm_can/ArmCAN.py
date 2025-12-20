   #initial commnands to set up the CAN node
import can
import time

import rclpy
from rclpy.node import Node
from all_interfaces.msg import ArmEndMotion
from all_interfaces.msg import EncoderArm

#TODO polling command section execution
#TODO test with PWM and direction 
#TODO decide on conditions in order to execute systems check commands and polling flags
#TODO decide on ROS2 architecture to take care of the above point 
#TODO run a script to setup CAN networks using ifconfig at system start, errors were most likely due to reinitialization
#TODO develop logic to send commands to the CAN bus only when there are nodes connected to it

BIT_RATE=500000
NUM_NODES_CH1=6 #number of nodes on channel B

SLAVE0_ID_CH1=0x256 #first node on channel 1
MASTER_ID=0x365

#loop to generate all the slave node addresses
SLAVE_IDS1= list(range(SLAVE0_ID_CH1, NUM_NODES_CH1+SLAVE0_ID_CH1))

#SLAVE_IDS=[0x256,0x257,0x258,0x259,0x260]

#lists to store if data has entered or not

#channel 1
upcheck1=[0]*NUM_NODES_CH1
datacheck1=[0]*NUM_NODES_CH1
CANCheck1=[0]*NUM_NODES_CH1
data1=[0]*NUM_NODES_CH1

DELAY=5000//NUM_NODES_CH1-50

#array to hold incoming 8 bytes of data for encoder from each node

#channel 1
encoderIn1= [[0]*6 for i in range(NUM_NODES_CH1)]

#channel 1
diff1=[0]*NUM_NODES_CH1
pos1=[0]*NUM_NODES_CH1 #absolute position
# angPos1=[0]*NUM_NODES_CH1 #angular position

#store PWM and direction channels for each motor
PWM1=[0]*NUM_NODES_CH1 #set 8 bit PWM value from 0 - 255
DIR1=[0]*NUM_NODES_CH1



#PWM[0]=20 #for testing, comment out later 
#DIR[0]=1

#PWM[1]=50
#DIR[1]=0

sysCheck=0 #000 -> 0
polling=7 #111->7  (start polling)
stopPoll=5 #101->7 (stop polling)
reset=1 #001 -> 1 (reset encoders)
PID_hold=3 #011 -> 3 (PID hold -> implies brakes are engaged)

#define the nodes of the arm here
base_node = 0x258
shoulder_node = 0x256
elbow_node = 0x257
wrist_node_1 = 0x259
wrist_node_2 = 0x25A

PID_hold_shoulder = False #we are holding using PID and brakes
PID_hold_elbow = False #we are holding using PID and brakes


class CAN_Publisher(Node):
    
    def __init__(self):
        super().__init__('Arm_Master')
        #update rate of main loop 
        timer_period = 1e-4

        self.sysCANCheckFlag1=1 #flag to perform sysCANCheck
        self.pollingFlag1=0 #flag to start polling mode
        self.resetMode=0 #flag to reset the encoders
        
        self.shoulder_PID_speed = 0
        self.shoulder_PID_dir = 0
        self.elbow_PID_speed = 0
        self.elbow_PID_dir = 0
        self.PID_threshold = 25

        
        self.subscription_arm=self.create_subscription(ArmEndMotion,'arm_commands',self.CAN_callback1,10)
        self.subscription_arm_PID =self.create_subscription(ArmEndMotion,'arm_commands_pid_shoulder',self.shoulder_callback,10)
        self.subscription_elbow_PID =self.create_subscription(ArmEndMotion,'arm_commands_pid_elbow',self.elbow_callback,10)
        

        self.publisher_encoder = self.create_publisher(EncoderArm,'encoder_arm',10)

        self.timer = self.create_timer(timer_period, self.timer_callback)
    
    def shoulder_callback(self,msg:ArmEndMotion):
        self.shoulder_PID_dir = msg.direction[0]
        self.shoulder_PID_speed = msg.speed[0]

    def elbow_callback(self,msg:ArmEndMotion):
        self.elbow_PID_dir = msg.direction[0]
        self.elbow_PID_speed = msg.speed[0]

    def CAN_callback1(self,msg:ArmEndMotion):    	
        #print("Received 1")        
        #print(len(msg.direction))
        #decide which topic to subscribe from 
        for i in range(len(msg.direction)):
            DIR1[i]=msg.direction[i]
            PWM1[i]=msg.speed[i]

            if PWM1[i] < self.PID_threshold:
                if i == shoulder_node - SLAVE0_ID_CH1:
                    DIR1[i] = self.shoulder_PID_dir
                    PWM1[i] = self.shoulder_PID_speed
                    global PID_hold_shoulder
                    PID_hold_shoulder =  True

                if i == elbow_node - SLAVE0_ID_CH1:
                    DIR1[i] = self.elbow_PID_dir
                    PWM1[i] = self.elbow_PID_speed
                    global PID_hold_elbow
                    PID_hold_elbow = True
            else:

                if i == shoulder_node - SLAVE0_ID_CH1:
                    PID_hold_shoulder =  False

                if i == elbow_node - SLAVE0_ID_CH1:
                    PID_hold_elbow = False


                self.shoulder_PID_dir=1
                self.shoulder_PID_speed=0
                self.elbow_PID_dir=1
                self.elbow_PID_speed=0

        if msg.reset:
            self.resetMode=1
            self.sysCANCheckFlag1=0
            self.pollingFlag1=0
        else:
            self.resetMode=0
            #print('here')
            if msg.sys_check: #set flags accordingly 
                self.sysCANCheckFlag1=1
                self.pollingFlag1=0
            else:
                self.sysCANCheckFlag1=0
                self.pollingFlag1=1



    def timer_callback(self):
        #msg=can.Message()
        if self.resetMode:
            #print('reset')
            resetEnc(msg,can1)
        if self.sysCANCheckFlag1:
            #print('sysCheck')
            sysCANCheck(msg,can1) #run systems check on all the CAN nodes
            out =  f"Status channel 1: {upcheck1}"
            self.get_logger().info(out)
            time.sleep(0.2) #small delay before moving to next command
            #sysCANCheckFlag=0
        if self.pollingFlag1:
            #print('poll')
            enc_msg=poll(msg,can1)
            self.publisher_encoder.publish(enc_msg)
            #if i<2000:
            #    #poll(msg,can0)
            #    i+=1
            #else:
            #    PWM[0]=0 
            #    PWM[1]=0
            
        delay_us(20)
              



def delay_us(micros):
    time.sleep(micros/1000000.0)

def pending1(msg_rx:can.Message):
    #msg_rx=can.Message
    #create a loop for checking messages
    #print(f"Entered loop {msg_rx.data}")
    if msg_rx.dlc==1: #a systems check command
        for i in range(NUM_NODES_CH1):
            if msg_rx.arbitration_id == SLAVE_IDS1[i]:
                datacheck1[i]=1
                data1[i]=msg_rx.data[0]

    #use the above logic for up-link check of CAN nodes


    if msg_rx.dlc==6:
        for i in range(NUM_NODES_CH1):
            if msg_rx.arbitration_id==SLAVE_IDS1[i]:
                CANCheck1[i]=1
                for j in range(6):
                    encoderIn1[i][j]=msg_rx.data[j] #copy the bytes received
    #use the above logic for bytes reconstruction

def resetEnc(msg_tx:can.Message,bus1):
    msg_tx.dlc=1 #send 1 byte of data for encoder reset
    query = [0]*8
    #run systems check on channel 0    
    for i in range(NUM_NODES_CH1):
        address=SLAVE_IDS1[i]-SLAVE0_ID_CH1 #returns an 8 bit number
        #as there are only 32 nodes maximum, the last 5 bits will be filled

        command=(reset<<5) | address; #sysCheck command created

        query[0]=command
        msg_tx.data=query
        bus1.send(msg_tx)
        #print(f"Message sent {msg_tx.data}")
        delay_us(10); #small delay before engaging the bus 



def sysCANCheck(msg_tx:can.Message,bus1):
    msg_tx.dlc=1 #send 1 byte of data to the slave nodes for CAN system check
    query=[0]*8 #create a query buffer of 8 bytes to send to the CAN bus   
    
    #run systems check on channel 1
    for i in range(NUM_NODES_CH1):
        upcheck1[i]=0 #reset the flag before checking
        address=SLAVE_IDS1[i]-SLAVE0_ID_CH1 #returns an 8 bit number
        #as there are only 32 nodes maximum, the last 5 bits will be filled

        command=(sysCheck<<5) | address #sysCheck command created

        query[0]=command
        msg_tx.data=query
        bus1.send(msg_tx)
        #print(f"Message sent {msg_tx.data}")


        time.sleep(5/1000.0)
        #wait 5 ms before checking for response

        if datacheck1[i]==1:
            #print(f"datacheck {i} = {datacheck[i]}")
            #mirroring back a 1 byte message, it should be same as command
            if data1[i]==command:
                upcheck1[i]=1 #this node is working correctly
                #print(f"Working {i}")

        datacheck1[i]=0
        data1[i]=0 #clean the data buffer
        time.sleep(1/1000.0) #wait 1 ms before testing the next node

def to_int16(binary_str):
    """Convert a 16-bit binary string to signed int16."""
    if len(binary_str) != 16:
        raise ValueError("The binary string must be 16 bits for int16_t conversion.")
    return int(binary_str, 2) if binary_str[0] == '0' else int(binary_str, 2) - (1 << len(binary_str))

def to_int32(binary_str):
    """Convert a 32-bit binary string to signed int32."""
    if len(binary_str) != 32:
        raise ValueError("The binary string must be 32 bits for int32_t conversion.")
    return int(binary_str, 2) if binary_str[0] == '0' else int(binary_str, 2) - (1 << len(binary_str))


def poll(msg_tx:can.Message,bus1):
    enc_msg=EncoderArm()
    msg_tx.dlc=3 #send command of length 3
    #along with PWM and direction signals
    query=[0]*8 #query to transmit to central node
    
    #polling for channel 1
    for j in range(len(query)):
        query[j]=0
    for i in range(NUM_NODES_CH1):
        #poll each CAN node and check if you receive a message back
        #a maximum of 32 nodes, which means last 5 bits to be used as address
        address=SLAVE_IDS1[i]-SLAVE0_ID_CH1; #returns an 8 bit number

        #as there are only 32 nodes maximum, the last 5 bits will be filled

        prefix = polling

        if (i == shoulder_node - SLAVE0_ID_CH1 and PID_hold_shoulder == True) or (i == elbow_node - SLAVE0_ID_CH1 and PID_hold_elbow == True):
            prefix = PID_hold
        else:
            prefix = polling
                

        command=(prefix<<5) | address; #polling command created

        query[0]=command; #0 for 550, 1 for 551, ...
        query[1]=PWM1[i]
        query[2]=DIR1[i]


        #send the CAN message
        msg_tx.data=query
        bus1.send(msg_tx)
         
        #wait 50 us before checking for message from slave node
        delay_us(50);
        if (CANCheck1[i]==1): #data received
            CANCheck1[i]=0; #reset the data flag

            b0 = encoderIn1[i][0]
            b1 = encoderIn1[i][1]
            b2 = encoderIn1[i][2]
            b3 = encoderIn1[i][3]

            
            bin0 = format(b0, '08b')
            bin1 = format(b1, '08b')
            bin2 = format(b2, '08b')
            bin3 = format(b3, '08b') 

            position = bin0 + bin1 + bin2+ bin3

            b4 = encoderIn1[i][4]
            b5 = encoderIn1[i][5]

            bin4 = format(b4,'08b')
            bin5 = format(b5,'08b')

            difference = bin4 + bin5    
            s_pos = to_int32(position)
            s_diff = to_int16(difference)
            
            pos1[i] = s_pos
            diff1[i] = s_diff


        (f"MSG, channel 1, {i},{pos1[i]},{diff1[i]}");

        delay_us(DELAY); #wait 1.25 ms =(5/4) ms before polling the next node (-50 for the previous delay of 20 us)
        #the polling interval is fixed here at 5 ms
    
    #converting the data into ROS2 topics
    enc_msg.arm_node0 = [pos1[0],diff1[0]]
    enc_msg.arm_node1 = [pos1[1],diff1[1]]
    enc_msg.arm_node2 = [pos1[2],diff1[2]]
    enc_msg.arm_node3 = [pos1[3],diff1[3]]
    enc_msg.arm_node4 = [pos1[4],diff1[4]]
    enc_msg.arm_node5 = [pos1[5],diff1[5]]
    return enc_msg
    
can1 = can.interface.Bus(channel = 'can1', interface = 'socketcan',receive_own_messages=False)
#prevent buffer overflow if one channel is disconnected

#printer=can.Printer()
#listener = msg_rx_routine                                             
#listener = pending                                         
notifier1 = can.Notifier(can1, [pending1])

#can.Notifier(can0, [pending(can)])
#msg = can.Message(is_extended_id=False, arbitration_id=0x123, data=[0, 1, 2, 3, 4, 5, 6, 7])

msg = can.Message(arbitration_id=MASTER_ID, is_extended_id=False, is_remote_frame=False)

def main(args=None):

    rclpy.init(args=args)
    master=CAN_Publisher()
        
    try:
        #os.system(f'sudo ip link set can0 type can bitrate {BIT_RATE}')
        #os.system('sudo ifconfig can0 up')
        #os.system('sudo ifconfig can0 txqueuelen 65536')
        rclpy.spin(master)            
    except KeyboardInterrupt:
        master.destroy_node() #kill node on interruption
        rclpy.shutdown()   
        #os.system('sudo ifconfig can0 down') #shutdown CAN0 network    
    except Exception as e:
        print(e)
    
if __name__=='main':
    main()
