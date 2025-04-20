import logging

import serial
import time
from threading import Lock
from scservo_sdk_waveshare import *

class ServoController:

    # Control table address
    ADDR_SCS_TORQUE_ENABLE = 40
    ADDR_SCS_GOAL_ACC = 41
    ADDR_SCS_GOAL_POSITION = 42
    ADDR_SCS_GOAL_SPEED = 46
    ADDR_SCS_PRESENT_POSITION = 56

    # Default settings
    ID_Horizontal_Servo = 2  # Horizontal servo ID
    ID_Vertical_Servo = 3  # Vertical servo ID
    BAUDRATE = 115200  # Driver board default baudrate : 115200
    DEVICENAME = 'COM3'  # Check your port name
    protocol_end = 1  # SCServo bit end(STS/SMS=0, SCS=1)

    # Servo 1 (ID 2) parameters
    ID_Horizontal_Servo_MIN_POSITION = 200  # Min position for servo 1
    ID_Horizontal_Servo_MAX_POSITION = 800  # Max position for servo 1

    # Servo 2 (ID 3) parameters
    ID_Vertical_Servo_MIN_POSITION = 700  # Min position for servo 2
    ID_Vertical_Servo_MAX_POSITION = 980  # Max position for servo 2

    # Common parameters
    SCS_MOVING_STATUS_THRESHOLD = 20  # Moving status threshold
    SCS_MOVING_SPEED = 50  # Moving speed
    SCS_MOVING_ACC = 0  # Moving acceleration
    MAX_RETRIES = 3  # Max read retries






    def __init__(self, port='COM3', baudrate=115200):
        self.serial = None
        self.lock = Lock()
        self.DEVICENAME = port
        self.BAUDRATE = baudrate

        # Initialize PortHandler instance
        self.portHandler = PortHandler(self.DEVICENAME)
        # Initialize PacketHandler instance
        self.packetHandler = PacketHandler(self.protocol_end)


        if self.portHandler.openPort():
            logging.info("Succeeded to open the port")
        else:
            logging.info("Failed to open the port")

        if self.portHandler.setBaudRate(self.BAUDRATE):
            logging.info("Succeeded to change the baudrate")
        else:
            logging.info("Failed to change the baudrate")

        logging.debug("Before sleep for boating up board in forwarding mode")
        time.sleep(5)
        logging.debug("After sleep for boating up board in forwarding mode")

        self.setup_servo(2,0,0)
        self.setup_servo(3,0,0)

    def setup_servo(self,servo_id, acc, speed):
        # Write servo acceleration

        scs_comm_result, scs_error = self.packetHandler.write1ByteTxRx(self.portHandler, servo_id, self.ADDR_SCS_GOAL_ACC, acc)
        if scs_comm_result != COMM_SUCCESS:
            print("%s" % self.packetHandler.getTxRxResult(scs_comm_result))
        elif scs_error != 0:
            print("%s" % self.packetHandler.getRxPacketError(scs_error))

        # Write servo speed
        scs_comm_result, scs_error = self.packetHandler.write2ByteTxRx(self.portHandler, servo_id, self.ADDR_SCS_GOAL_SPEED, speed)
        if scs_comm_result != COMM_SUCCESS:
            print("%s" % self.packetHandler.getTxRxResult(scs_comm_result))
        elif scs_error != 0:
            print("%s" % self.packetHandler.getRxPacketError(scs_error))



    def read_servo_position(self,servo_id):
        for attempt in range(self.MAX_RETRIES):
            try:
                scs_present_position_speed, scs_comm_result, scs_error = self.packetHandler.read4ByteTxRx(
                    self.portHandler, servo_id, self.ADDR_SCS_PRESENT_POSITION)

                if scs_comm_result == COMM_SUCCESS and scs_error == 0:
                    return scs_present_position_speed, scs_comm_result, scs_error

                print(
                    f"Read attempt {attempt + 1} failed for servo {servo_id}: {self.packetHandler.getTxRxResult(scs_comm_result)}")
                time.sleep(0.2)

            except Exception as e:
                print(f"Exception reading servo {servo_id}: {str(e)}")
                time.sleep(0.2)

        return 0, COMM_RX_FAIL, 0



    def __del__(self):
        if self.serial and self.serial.is_open:
            self.serial.close()

    def straighten_out(self):
        return self.move_servos(500,970)

    def move_servos(self,param1, param2):
        return self.move_servo(2,param1),self.move_servo(3,param2)


    def move_servo(self,servo_id, goal_position):
        # Write goal position
        scs_comm_result, scs_error = self.packetHandler.write2ByteTxRx(self.portHandler, servo_id, self.ADDR_SCS_GOAL_POSITION,
                                                                  goal_position)
        if scs_comm_result != COMM_SUCCESS:
            print("%s" % self.packetHandler.getTxRxResult(scs_comm_result))
            return False
        elif scs_error != 0:
            print("%s" % self.packetHandler.getRxPacketError(scs_error))
            return False

        # Wait until movement is complete
        start_time = time.time()
        while time.time() - start_time < 3:  # Timeout after 3 seconds
            # time.sleep(0.2)

            scs_present_position_speed, scs_comm_result, scs_error = self.read_servo_position(servo_id)

            if scs_comm_result != COMM_SUCCESS:
                print(self.packetHandler.getTxRxResult(scs_comm_result))
                continue
            elif scs_error != 0:
                print(self.packetHandler.getRxPacketError(scs_error))
                continue

            scs_present_position = SCS_LOWORD(scs_present_position_speed)
            scs_present_speed = SCS_HIWORD(scs_present_position_speed)

            print("[ID:%03d] GoalPos:%03d PresPos:%03d PresSpd:%03d"
                  % (servo_id, goal_position, scs_present_position, SCS_TOHOST(scs_present_speed, 15)))

            if not (abs(goal_position - scs_present_position) > self.SCS_MOVING_STATUS_THRESHOLD):
                return True

        print(f"Timeout waiting for servo {servo_id} to reach position")
        return False