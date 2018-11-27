"""
Declaring the data exchange format between robot and controller.
"""

import struct


def time2int(float_time):
    """
    Convert time from float (seconds) to int (nanoseconds).
    """
    int_time = int(float_time * 10000000)
    return int_time


def time2float(int_time):
    """
    Convert time from int (nanoseconds) to float (seconds).
    """
    float_time = float(int_time / 10000000)
    return float_time

def sensor2int(float_sensor):
    """
    Convert sensor data from float to int.
    """
    int_time = int(float_sensor * 1000000)
    return int_time

def sensor2float(int_sensor):
    """
    Convert sensor data from int to float.
    """
    float_time = float(int_sensor / 1000000)
    return float_time

NO_OF_DELAYS = 1
NO_OF_SIM_STEPS = 10
NO_OF_VOLTAGES_PER_DELAY = 2 * (1 + 10)
H2R_PACKET_FORMAT = 'qq%sq' % (NO_OF_DELAYS * NO_OF_VOLTAGES_PER_DELAY)
H2R_PACKET_SIZE = struct.calcsize(H2R_PACKET_FORMAT)
class H2R_PACKET_VARS:
    """
    Packet format for transmitting a number of actuation values from controller to robot.
    """
    Seq_number = 0
    Timestamp = 1


R2H_PACKET_FORMAT = 'qqqqqqqqqqq'
R2H_PACKET_SIZE = struct.calcsize(R2H_PACKET_FORMAT)
class R2H_PACKET_VARS:
    """
    Packet format for transmitting sensor data from robot to computer.
    """
    Seq_number = 0
    Tsr_k = 1
    Tsstx_k = 2
    Tasrx_k = 3
    Taw_k = 4
    Gyro_rate = 5
    Motor_position_left = 6
    Motor_position_right = 7
    Gyro_offset = 8
    Motor_voltage_applied_left = 9
    Motor_voltage_applied_right = 10
