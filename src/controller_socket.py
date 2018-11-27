"""
Socket for connecting the controller to the robot.
Handles timestamping, logging, and sending.
"""

import socket
import struct
import logging
import time
import traceback

import packet
import stats

MAX_QUEUE_LEN = 10


class ControllerSocket:
    """
    UDP socket between the Controller and the Robot
    """

    def __init__(self, ip_address, aport, sport, measurement_folder):
        self.ip_address = ip_address
        self.robot_port = aport
        self.sport = sport
        self.robot_ip = ip_address

        self.sock = socket.socket(socket.AF_INET, socket.SOCK_DGRAM)

        self.stat = stats.Stats(subfolder=measurement_folder)
        self.reord_cntr = self.stat.new_counter("controller_reord")
        self.lost_cntr = self.stat.new_counter("controller_lost")
        self.rx_cntr = self.stat.new_counter("controller_rx")
        self.tx_cntr = self.stat.new_counter("controller_tx")

        self.avg_single_hop_delay = 0

        self.tsr_ks = {}
        self.tsstx_ks = {}

        self.tssrx_ks = {}
        self.tastx_ks = {}

        self.tasrx_ks = {}
        self.taw_ks = {}

        self.gyro_rate_logger = self.stat.new_logger("gyro_rate_logger")
        self.motor_position_left_logger = self.stat.new_logger("motor_position_left_logger")
        self.motor_position_right_logger = self.stat.new_logger("motor_position_right_logger")
        self.gyro_offset_logger = self.stat.new_logger("gyro_offset_logger")
        self.motor_voltage_applied_left_logger = self.stat.new_logger("motor_voltage_applied_left_logger")
        self.motor_voltage_applied_right_logger = self.stat.new_logger("motor_voltage_applied_right_logger")
        self.robot_roundtrip_network_delay_logger = self.stat.new_logger("robot_roundtrip_network_delay_logger")
        self.controller_processing_delay_logger = self.stat.new_logger("controller_processing_delay_logger")

        self.tsr_k_logger = self.stat.new_logger("tsr_k_logger")
        self.tsstx_k_logger = self.stat.new_logger("tsstx_k_logger")

        self.tssrx_k_logger = self.stat.new_logger("tssrx_k_logger")
        self.tastx_k_logger = self.stat.new_logger("tastx_k_logger")

        self.tasrx_k_logger = self.stat.new_logger("tasrx_k_logger")
        self.taw_k_logger = self.stat.new_logger("taw_k_logger")

        logging.debug("Listen on address %s (port %s)", ip_address, str(self.sport))
        self.sock.setblocking(False)
        self.sock.bind(('', sport))

        self.old_timestamp = -1
        self.old_seq_number = 0

    @staticmethod
    def read_control_measurements(measurement):
        """
        Extracts the measurement values from the received measurement packet
        :param measurement: Received measurement packet
        :return: Measurement values
        """
        seq_number = measurement[packet.R2H_PACKET_VARS.Seq_number]
        gyro_rate = packet.sensor2float(measurement[packet.R2H_PACKET_VARS.Gyro_rate])
        enc_l = packet.sensor2float(measurement[packet.R2H_PACKET_VARS.Motor_position_left])
        enc_r = packet.sensor2float(measurement[packet.R2H_PACKET_VARS.Motor_position_right])
        gyro_offset = packet.sensor2float(measurement[packet.R2H_PACKET_VARS.Gyro_offset])
        motorVoltageApplied_left = packet.sensor2float(measurement[packet.R2H_PACKET_VARS.Motor_voltage_applied_left])
        motorVoltageApplied_right = packet.sensor2float(measurement[packet.R2H_PACKET_VARS.Motor_voltage_applied_right])

        return seq_number, gyro_rate, enc_l, enc_r, gyro_offset, motorVoltageApplied_left, motorVoltageApplied_right

    def receive(self):
        """
        Receive the sensor values of the robot.
        """
        try:
            while True:
                while True:
                    try:
                        dat, (ip, _) = self.sock.recvfrom(packet.R2H_PACKET_SIZE)
                        break
                    except socket.error:
                        continue
                tssrx_k = time.perf_counter()

                if self.robot_ip == '0.0.0.0':
                    # set ip addr to the addr from which sensor data was received
                    self.robot_ip = ip

                if len(dat) is packet.R2H_PACKET_SIZE:
                    self.rx_cntr.inc()

                    data = struct.unpack(packet.R2H_PACKET_FORMAT, dat)
                    seq_number, gyro_rate, enc_l, enc_r,\
                    gyro_offset, motorVoltageApplied_left, motorVoltageApplied_right = self.read_control_measurements(data)

                    self.gyro_rate_logger.timestamp(timestamp=tssrx_k, value=gyro_rate)
                    self.motor_position_left_logger.timestamp(timestamp=tssrx_k, value=enc_l)
                    self.motor_position_right_logger.timestamp(timestamp=tssrx_k, value=enc_r)
                    self.gyro_offset_logger.timestamp(timestamp=tssrx_k, value=gyro_offset)
                    self.motor_voltage_applied_left_logger.timestamp(timestamp=tssrx_k, value=motorVoltageApplied_left)
                    self.motor_voltage_applied_right_logger.timestamp(timestamp=tssrx_k, value=motorVoltageApplied_right)

                    # sequence number evaluation
                    if seq_number == self.old_seq_number + 1:
                        # everything is shiny proceed
                        pass
                    elif seq_number > self.old_seq_number + 1:
                        # we are receiving newer packet than expected -> lost packets
                        logging.debug("seqnr: %s, oldseqnr: %s", seq_number, self.old_seq_number)
                        self.lost_cntr.inc()
                    else:
                        # seq_number is smaller than already received one -> out of order
                        self.reord_cntr.inc()
                        # throw away out of order packets
                        continue

                    self.old_seq_number = seq_number

                    # Timestamp extraction
                    self.tssrx_ks[seq_number + 1] = tssrx_k
                    if data[packet.R2H_PACKET_VARS.Tasrx_k] is not 0:
                        self.tsr_ks[seq_number] = packet.time2float(data[packet.R2H_PACKET_VARS.Tsr_k])
                        self.tsstx_ks[seq_number] = packet.time2float(data[packet.R2H_PACKET_VARS.Tsstx_k])
                        self.tasrx_ks[seq_number] = packet.time2float(data[packet.R2H_PACKET_VARS.Tasrx_k])
                        self.taw_ks[seq_number] = packet.time2float(data[packet.R2H_PACKET_VARS.Taw_k])

                    x = [seq_number, gyro_rate, enc_l, enc_r,
                         gyro_offset, motorVoltageApplied_left, motorVoltageApplied_right]
                    return x

                else:
                    logging.debug("wrong packet size")

        except socket.timeout:
            logging.error('Rx timeout')
        except socket.error:
            logging.error('Rx error')
        except IndexError:
            logging.error('Index error')
            traceback.print_exc()
        except KeyboardInterrupt:
            self.stat.stop_logging()
            logging.error('Closing socket')
            self.sock.close()
            return

    def send(self, seq_number, *voltage_list):
        """
        Send a list of voltages to the robot.
        :param seq_number:   sequence number of the current sensor data packet
        :param voltage_list: list of voltages to be sent to the robot
        """

        timestamp = time.perf_counter()
        volt_list = list()
        for volt in voltage_list:
            volt_list.append(volt)
        try:
            self.sock.sendto(struct.pack(packet.H2R_PACKET_FORMAT, seq_number, packet.time2int(timestamp), *volt_list),
                             (self.robot_ip, self.robot_port))

            self.tx_cntr.inc()

            self.tastx_ks[seq_number + 1] = timestamp

            try:
                tsr_k = self.tsr_ks[seq_number]
                tsstx_k = self.tsstx_ks[seq_number]
                tssrx_k = self.tssrx_ks[seq_number]
                tastx_k = self.tastx_ks[seq_number]
                tasrx_k = self.tasrx_ks[seq_number]
                taw_k = self.taw_ks[seq_number]

                if packet.time2int(tasrx_k) is not 0:
                    self.tsr_k_logger.timestamp(timestamp=tsr_k, value=tsr_k)
                    self.tsstx_k_logger.timestamp(timestamp=tsstx_k, value=tsstx_k)

                    self.tssrx_k_logger.timestamp(timestamp=tssrx_k, value=tssrx_k)
                    self.tastx_k_logger.timestamp(timestamp=tastx_k, value=tastx_k)

                    self.tasrx_k_logger.timestamp(timestamp=tasrx_k, value=tasrx_k)
                    self.taw_k_logger.timestamp(timestamp=taw_k, value=taw_k)

                    del self.tsr_ks[seq_number - 1]
                    del self.tsstx_ks[seq_number - 1]

                    del self.tasrx_ks[seq_number - 1]
                    del self.taw_ks[seq_number - 1]

            except KeyError:
                logging.debug("Packet not found")

        except socket.error:
            logging.error('Tx error')
        return
