# MOOG Control Class
# Author: Hudson Burke
import serial
import time
import rclpy
from rclpy.node import Node
# taskkill /F /IM python.exe
class MOOG(Node):

    def __init__(self):

        super().__init__('moog_communication')
        self.get_logger().info("Initializing MOOG Communication Node")

        # Serial port configuration
        self.ser = serial.Serial(
            port = 'COM3',
            baudrate = 57600,							# Serial port transfer rate (bits/s)
            parity = serial.PARITY_NONE,
            stopbits = serial.STOPBITS_ONE,
            bytesize = serial.EIGHTBITS,
            timeout = 0.002 
        )
        self.ser.reset_output_buffer()

        self.command_types_hex = {
            'ESTOP':                0xe6, 
            'DISABLE':              0xdc, 
            'PARK':                 0xd2, 
            'LOW LIMIT ENABLE':     0xc8, 
            'LOW LIMIT DISABLE':    0xbe, 
            'ENGAGE':               0xb4, 
            'START':                0xaf, 
            'LENGTH MODE':          0xac, 
            'DOF MODE':             0xaa, 
            'RESET':                0xa0, 
            'INHIBIT':              0x96, 
            'RESERVED':             0x8c, 
            'NEW POSITION':         0x82
        }

        # Initial command to begin communication with the platform
        self._command = b"\xff\x82\x43\x00\x04\x00\x04\x00\x04\x00\x04\x00\x04\x00\x04\x29\x00"
        self._prev_command = self._command
        self._command_buffer = []
        self.state = 'POWERUP'
        self.states = {
            0b0000 : 'POWERUP', # Also POWER_UP in manual
            0b0001 : 'IDLE',
            0b0010 : 'STANDBY',
            0b0011 : 'ENGAGED',
            0b0100 : '(NOT USED)',
            0b1000 : 'PARKING',
            0b1001 : 'FAULT2',
            0b1010 : 'FAULT3',
            0b1011 : 'DISABLED',
            0b1100 : 'INHIBITED',       
        }

        self.text_output = ""
        
        self.current_actuator_commands = [1024, 1024, 1024, 1024, 1024, 1024]
        self._initialized = False

    def communication_loop(self): # publish current command at 60 Hz
        if len(self._command_buffer) and self._initialized:
            self._prev_command = self._command
            self._command = self._command_buffer.pop(0)
            # print("Received:" +  str(int((self._command >> 64) & 0xFFFF)))
        self.communicate()

    def communicate(self):
        self.ser.write(self._command)					            # Write current command to platform base
        bytesToRead = 20                                            # Response data is 20 bytes
        response_bytes : bytes = self.ser.read(bytesToRead)         # Read full platform status message
        response_hex_string : str = response_bytes.hex(':')     
        self.response = response_hex_string.split(':')              #TODO: Automatically convert hex strings to int
        try:
        # parse through 20 bytes read from serial port
            self.frame_sync =                   self.response[0]
            self.fault_data_1 =                 self.response[1]
            self.fault_data_2 =                 self.response[2]
            self.discrete_io_info =             self.response[3]
            self.checksum =                     self.response[4]     
            self.actuator_a_low_feedback =      self.response[5] 
            self.actuator_a_high_feedback =     self.response[6]
            self.actuator_b_low_feedback =      self.response[7]
            self.actuator_b_high_feedback =     self.response[8]
            self.actuator_c_low_feedback =      self.response[9]
            self.actuator_c_high_feedback =     self.response[10]
            self.actuator_d_low_feedback =      self.response[11]
            self.actuator_d_high_feedback =     self.response[12]
            self.actuator_e_low_feedback =      self.response[13]
            self.actuator_e_high_feedback =     self.response[14]
            self.actuator_f_low_feedback =      self.response[15]	
            self.actuator_f_high_feedback =     self.response[16]
            self.machine_state_info =           self.response[17]
            self.motion_base_id_low =           self.response[18]
            self.motion_base_id_high =          self.response[19]
        except:
            # print("Response Error")
            pass

        try:
            machine_state_int = int(self.machine_state_info, 16)
            self.state = self.states[machine_state_int & 0b1111] # use last 4 bits to look up state 
            self.mode = (machine_state_int >> 4) & 1
        except: 
            # print("State Read Error")
            pass


    # command_type string corresponding to hex in dict ; commands list of ints
    def build_frame(self, command_type : str, commands=[0,0,0,0,0,0]):
        command_prefix_hex = 0xff  # frame sync
        command_machine_id_low = 0x29 # IDK where these came from
        command_machine_id_high = 0x00

        command_type_hex = self.command_types_hex[command_type]

        # Split commands into low and high bytes in hex (high bytes must have msb of 0)
        commands_hex = []
        for command in commands:
            command &= 0x7FFF
            commands_hex.append(command & 0xff) # low byte
            commands_hex.append(command >> 8)  # high byte

        # Add bytes 1 and 3-16, limit to 8 bits, then zero MSB
        checksum = command_type_hex + sum(commands_hex) + command_machine_id_low + command_machine_id_high
        checksum &= 0b01111111

        # Final command to send in hex
        frame = bytearray([command_prefix_hex, command_type_hex, checksum, *commands_hex, command_machine_id_low, command_machine_id_high])
        return frame


    def initialize_platform(self): # TODO: handle errors
        # Initial command
        while self.state != 'IDLE':
            print(self.state)
            # if self.state == 'FAULT2' or self.state == 'FAULT3' or self.state == 'INHIBITED':
            self.reset()
            time.sleep(1)
            self.command_length()
            print("Stuck in fault mode")
            
        # Change to DOF mode
        while not self.mode: 
            self.dof_mode()
            time.sleep(1)
            print("Stuck in length mode")
        
        # Continue to send 
        self.command_dof()
        time.sleep(1)

        print("Engaging")        
        # Engage
        while self.state != 'STANDBY':
            self.engage()
            time.sleep(1/60)

        # Ready to accept new positions
        while self.state != 'ENGAGED':
            time.sleep(float(2/6000))
            self.command_dof()
        print("Engaged")
        self._initialized = True

    def e_stop(self):
        if self.state != 'IDLE':
            self.command('ESTOP')

    def disable(self):
        self.command('DISABLE')
        self.text_output = 'Base Disabled. Please remove & re-apply power to reset.'
        
    def park(self):
        # Park
        if self.state == 'ENGAGED' or self.state == 'STANDBY':
            while self.state != "PARKING" or self.state != "IDLE":
                self.command('PARK') # TODO: actuators need to be same length as last command, maybe?
        else:
            self.text_output = 'PARK valid only in ENGAGED, STANDBY states'

    def low_limit_enable(self):
        self.command('LOW LIMIT ENABLE')

    def low_limit_disable(self):
        self.command('LOW LIMIT DISABLE')

    def engage(self):
        if self.state == 'IDLE':
            if self.mode:
                self.text_output = 'Engaging in DOF MODE...'
                commands = [16383, 16383, 29000, 16383, 16383, 16383]
            else: 
                self.text_output = 'Engaging in Length Mode...'
                commands = [1024, 1024, 1024, 1024, 1024, 1024]
            self.command('ENGAGE', commands)
        else:
            self.text_output = 'ENGAGE valid only in the IDLE state'

    def is_engaged(self):
        return self.state == "ENGAGED"

    # Same as ENGAGE, except the user may define the starting position of the base.
    def start(self, starting_position=[]): #TODO: differentiate between length and DOF mode and add limits
        if self.state == 'IDLE':
            self.command('START', starting_position)
        else:
            self.text_output = 'START valid only in the IDLE state'

    def length_mode(self):
        if self.state == 'IDLE' or self.state == 'POWERUP':
            self.command('LENGTH MODE', [1024, 1024, 1024, 1024, 1024, 1024])
        else:
            self.text_output = 'LENGTH MODE valid only in IDLE, POWERUP states'

    def dof_mode(self):
        if self.state == 'IDLE' or self.state == 'POWERUP':
            self.command('DOF MODE', [16383, 16383, 29000, 16383, 16383, 16383])
        else:
            self.text_output = 'LENGTH MODE valid only in IDLE, POWERUP states'

    def reset(self):
        # if self.state == 'FAULT2' or self.state == 'FAULT3' or self.state == 'INHIBITED':
        self.command('RESET')
        print("yep")

    def inhibit(self):
        if self.state == 'POWERUP' or self.state == 'IDLE':
            self.command('INHIBIT')
            self.text_output = 'Base will ignore all other commands until next RESET'
        else:
            self.text_output = 'INHIBIT valid only in IDLE, POWERUP states'

    def command_dof(self, roll=16383, pitch=16383, heave=29000, surge=16383, yaw=16383, lateral=16383, buffer=False):
        if self.mode:
            self.command('NEW POSITION', [roll, pitch, heave, surge, yaw, lateral], buffer=buffer)
        else:
            self.text_output = 'Command rejected: Base currently in Length Mode'

    def command_dof_degrees(self, roll=0, pitch=0, heave=29000, surge=16383, yaw=0, lateral=16383, buffer=False):
        roll = max(min(roll, 29), -29)
        pitch = max(min(pitch, 33), -33)
        yaw = max(min(yaw, 29), -29)

        roll = max(int(32767/58 * (roll + 29)), 0)
        pitch = max(int(32767/66 * (pitch + 33)), 0)
        yaw = max(int(32767/58 * (yaw + 29)), 0)
        
        self.command_dof(roll, pitch, heave, surge, yaw, lateral, buffer)
    
    def command_length(self, a=1024, b=1024, c=1024, d=1024, e=1024, f=1024):
        if not self.mode:
            self.command('NEW POSITION', [a, b, c, d, e, f])
        else:
            self.text_output = 'Command rejected: Base currently in DOF Mode'

    def set_command(self, new_command):
        # TODO: Lock
        self._prev_command = self._command
        self._command = new_command


    def command(self, command_type, commands=None, buffer=False):
        if commands is None:
            self.text_output = 'No actuator values provided. Will use default for current mode...'
            if self.mode: # checks for DOF mode
                commands = [16383, 16383, 29000, 16383, 16383, 16383]
            else: 
                commands = [1024, 1024, 1024, 1024, 1024, 1024]
        frame = self.build_frame(command_type, commands)
        if buffer:
            self._command_buffer.append(frame)
        else:
            self.set_command(frame)

    

# If launched as main, initialize the platform, subscribe to topic for commands, publish current state, and 
def main(args=None):
    rclpy.init(args=args)
    moog = MOOG()
    
    # Initialize platform
    moog.initialize_platform()

    # ROS Rate 60 Hz
    timer_period = 1/60
    timer = moog.create_timer(timer_period, moog.communication_loop)

    # Subscribe to topic "moog_cmds" for commands
    moog.create_subscription(String, 'moog_cmds', moog.command_callback, 10)

    # Publish current state to topic "moog_state"
    moog_state_pub = moog.create_publisher(String, 'moog_state', 10)

    # Destroy and shutdown
    moog.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()