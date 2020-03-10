import struct
import datetime
import time
import socket
import argparse
import logging
import os
import traceback
import controller_params as p

import ev3dev.ev3 as ev3
import packet


# ----------- Sensor Methods -----------------
def send_sensor_signals(socket, addr, port,
                        tsr_k, tsstx_k, tasrx_k, taw_k,
                        seq_number, gyro_rate, motor_pos_left, motor_pos_right, gyro_offset,
                        motorVoltageApplied_left, motorVoltageApplied_right):
    """ Packs all sensory information into predefined packet format and sends over UDP connection

    :param socket: UDP socket used to send packets
    :param timestamp: Clock-time of the current packet
    :param seq_number: Sequence number of the current number
    :param gyro_rate: Gyro sensor measurement
    :param motor_pos_left: Left motor position
    :param motor_pos_right:  Right motor position
    :return: None
    """
    logging.debug("Sending tsrk: %f tsstx_k: %f tasrx_k: %f taw_k: %f", tsr_k, tsstx_k, tasrx_k, taw_k)
    stuff = struct.pack(packet.R2H_PACKET_FORMAT, seq_number,
                        packet.time2int(tsr_k), packet.time2int(tsstx_k), packet.time2int(tasrx_k), packet.time2int(taw_k),
                        packet.sensor2int(gyro_rate), packet.sensor2int(motor_pos_left), packet.sensor2int(motor_pos_right),
                        packet.sensor2int(gyro_offset), packet.sensor2int(motorVoltageApplied_left), packet.sensor2int(motorVoltageApplied_right))
    socket.sendto(stuff, (addr, port))


# For reading from the sensor files
def SensorRead(infile):
    infile.seek(0)
    return float(infile.read().decode().strip())


def calibrate_gyro(gyroSensorValueRaw):

    gyroOffset = 0.0
    gyroRateCalibrateCount = 200
    for i in range(gyroRateCalibrateCount):

        gyro = SensorRead(gyroSensorValueRaw)
        logging.debug("S: Gyro reading %d: %f", i, gyro)

        gyroOffset = gyroOffset + gyro
        time.sleep(0.01)

    gyroOffset = gyroOffset / gyroRateCalibrateCount

    return gyroOffset


def init_sensors():
    """ Instantiates and calibrates both gyro sensor and motors

    :return: Gyro and EV3Motors instances
    """
    # Create EV3 resource objects
    gyroSensor = ev3.GyroSensor()
    gyroSensor.mode = gyroSensor.MODE_GYRO_RATE

    motorLeft = ev3.LargeMotor('outC')
    motorRight = ev3.LargeMotor('outB')

    # Open sensor and motor files
    gyroSensorValueRaw = open(gyroSensor._path + "/value0", "rb")

    # Reset the motors
    motorLeft.reset()
    motorRight.reset()
    motorLeft.run_direct()
    motorRight.run_direct()

    motorEncoderLeft = open(motorLeft._path + "/position", "rb")
    motorEncoderRight = open(motorRight._path + "/position", "rb")

    return gyroSensorValueRaw, motorEncoderLeft, motorEncoderRight

# ----------- Actuator Methods -----------------


# For writing to motor files
def MotorWrite(outfile, value):
    outfile.truncate(0)
    outfile.write(str(int(value)))
    outfile.flush()


# Function to set the duty cycle of the motors
def SetDuty(motorDutyFileHandle, voltage):
    # Voltage to PWM and cast to int
    duty = int(round(voltage*100/8.087))

    # Clamp the value between -100 and 100
    if duty > 0:
        duty = min(100, duty)
    elif duty < 0:
        duty = max(-100, duty)

    # Apply the signal to the motor
    MotorWrite(motorDutyFileHandle, duty)


def init_actuators():
    """ Instantiates and calibrates both gyro sensor and motors

    :return: Gyro and EV3Motors instances
    """
    motorLeft = ev3.LargeMotor('outC')
    motorRight = ev3.LargeMotor('outB')

    # Reset the motors
    motorLeft.reset()
    motorRight.reset()
    motorLeft.run_direct()
    motorRight.run_direct()

    motorDutyCycleFile_left = open(motorLeft._path + "/duty_cycle_sp", "w")
    motorDutyCycleFile_right = open(motorRight._path + "/duty_cycle_sp", "w")

    return motorDutyCycleFile_left, motorDutyCycleFile_right

# ----------- Main Loop -----------------

def main(ts, c_addr, s_port, a_port, log_enabled):
    """ Main function called from __main__
    :param ts: Sampling period in milliseconds
    :param c_addr: IP address of the controller
    :param s_port: Port number to send sensor signals
    :param a_port: Port number to receive actuation signals
    :param log_enabled: Set to 'True' to activate logging sensor measurements & states into logfile
    :return: None
    """

    buttons = ev3.Button()
    leds = ev3.Leds()

    leds.set_color(leds.LEFT, leds.RED)
    leds.set_color(leds.RIGHT, leds.RED)

    logging.info("Lay down the robot. Press the up button to start calibration. Press the down button to exit.")

    # Initialization of the sensors
    gyroSensorValueRaw, \
    motorEncoderLeft, motorEncoderRight = init_sensors()

    logging.debug("Initialized sensors")

    # Initialization of the actuators
    motorDutyCycleFile_left, motorDutyCycleFile_right = init_actuators()

    logging.debug("Initialized actuators")

    # Wait for up or down button
    while not buttons.up and not buttons.down:
        time.sleep(0.01)

    # If the up button was pressed wait for release and start calibration
    while buttons.up:
        time.sleep(0.01)

    # Calibration
    gyroOffset = calibrate_gyro(gyroSensorValueRaw)

    logging.info("Calibration done. Start the Controller and lift me!")
    ev3.Sound.beep().wait()
    leds.set_color(leds.LEFT, leds.AMBER)
    leds.set_color(leds.RIGHT, leds.AMBER)

    # Initialization of the sensor sockets
    udp_socket_sensor = socket.socket(socket.AF_INET, socket.SOCK_DGRAM)
    udp_socket_sensor.setblocking(0)
    udp_socket_sensor.bind(('', s_port))
    logging.debug("Initialized sensor UDP socket")

    # Initialization of the actuator socket
    udp_socket_actuator = socket.socket(socket.AF_INET, socket.SOCK_DGRAM)
    udp_socket_actuator.setblocking(1)
    udp_socket_actuator.settimeout(0.001)
    udp_socket_actuator.bind(('', a_port))

    logging.debug("Initialized actuator UDP socket")

    # Inits:
    next_trigger_time = 0  # he "starts" the loop sending the first message
    k = 0

    # Aux variables
    first_packet = True
    buttons = ev3.Button()
    seq_no_applied = 0
    motor_voltage_applied_left = 0
    motor_voltage_applied_right = 0
    prediction_count = 0
    receptionStarted = False
    no_of_sim_steps = 10
    voltagePredictions = [0]*2*no_of_sim_steps
    tau_RTT = 0

    avg_diff = p.SAMPLING_TIME
    diff_variance = 0
    old_sens_timestamp = time.perf_counter()

    tsr_k = time.perf_counter()

    tsstx_k = time.perf_counter()

    tasrx_k = 0

    taw_k = 0

    # Robot logging
    timenow = datetime.datetime.now()
    if log_enabled:
        filename = "datalog" +str(timenow.year)+"-"+ str(timenow.month)+"-"+str(timenow.day)+"_"+ str(timenow.hour)+ "_"+str(timenow.minute)+ "_"+str(timenow.second)+ ".csv"
        f = open(filename, "w")
        f.write("gyro,enc_l,enc_r, rtt, prediction_count\n")
    outdated_warning_printed = False

    while True:
        next_trigger_time = time.perf_counter() + ts

        k += 1

        # Sensor readings
        tsr_km1 = tsr_k
        tsr_k = time.perf_counter()
        gyro = SensorRead(gyroSensorValueRaw)
        new_sens_timestamp = time.perf_counter()

        enc_l = SensorRead(motorEncoderLeft)
        enc_r = SensorRead(motorEncoderRight)

        diff_sens_timestamp = new_sens_timestamp - old_sens_timestamp
        old_sens_timestamp = new_sens_timestamp
        avg_diff = ((k-1) / k) * avg_diff + 1/k * diff_sens_timestamp
        diff_variance = ((k - 1) / k) *  diff_variance + (1/k)*(diff_sens_timestamp - avg_diff) *  (diff_sens_timestamp - avg_diff)

        # Sensor transmission
        try:
            tsstx_km1 = tsstx_k
            tsstx_k = time.perf_counter()  # transmission of a time reply for the controller #TODO in the send_sensor_signal?
            send_sensor_signals(udp_socket_sensor, c_addr, s_port,
                                tsr_km1, tsstx_km1, tasrx_k, taw_k,
                                k, gyro, enc_l, enc_r, gyroOffset, motor_voltage_applied_left,
                                motor_voltage_applied_right)

            if receptionStarted:
                logging.debug("Sensing   %d sent     at %f with actuation %f", k, tsstx_k, tasrx_k)


             # logging.debug("Sensing %d sent at %f" % (k, t0_req_tx_current))

        except KeyboardInterrupt:
            udp_socket_sensor.close()
            return
        except socket.error:
            return
        # Take timestamp after packet was sent
        timestamp = time.perf_counter()
        # Actuation reception - Use the rest of the time to receive host packets, trying at least once.


        while True:
            rx_packet = None
            try:
                while True:
                    rx_bytes = udp_socket_actuator.recv(packet.H2R_PACKET_SIZE)
                    rx_packet = rx_bytes[:]
            except socket.timeout:
                if rx_packet is not None:
                    if first_packet:
                        logging.info("Control loop started.")
                        leds.set_color(leds.LEFT, leds.GREEN)
                        leds.set_color(leds.RIGHT, leds.GREEN)
                        first_packet = False
                    tasrx_k = time.perf_counter()  # reception of the time request from the controller
                    logging.debug("Actuation %d received at %f" % (k, tasrx_k))
                    receptionStarted = True
                    data_from_host = struct.unpack(packet.H2R_PACKET_FORMAT, rx_packet)
                    seq_no_received = data_from_host[packet.H2R_PACKET_VARS.Seq_number]
                    # If recieved packet is newer than the one last applied:
                    if seq_no_received == k:
                        # Measure round trip time and get appropriate index
                        tau_RTT = time.perf_counter() - timestamp
                        # Get voltages
                        # Current:
                        voltage_left = float(data_from_host[2])/1000000
                        voltage_right = float(data_from_host[3])/1000000
                        # Predictions:
                        for i in range(no_of_sim_steps):
                            voltagePredictions[2*i] = float(data_from_host[2*i+2])/1000000
                            voltagePredictions[2*i+1] = float(data_from_host[2*i+3])/1000000
                        # Reset prediction counter
                        prediction_count = 0
                        motor_voltage_applied_left = voltage_left
                        motor_voltage_applied_right = voltage_right
                        time_last_applied = time.perf_counter()
                        seq_no_applied = seq_no_received
                        while time.perf_counter() <= next_trigger_time-0.003:
                            pass
                        if log_enabled:
                            f.write("%f,%f,%f,%f,%f\n" % (gyro, enc_l, enc_r, tau_RTT, prediction_count))
                        SetDuty(motorDutyCycleFile_left, voltage_left)
                        SetDuty(motorDutyCycleFile_right, voltage_right)
                        taw_k = time.perf_counter()
                        break
 
                else:
                    pass
                    # logging.debug('Actuator: Not received %d at %f' % (k, time.perf_counter()))

            except KeyboardInterrupt:
                udp_socket_actuator.close()
                logging.debug("Keyboard interrupt, exiting")
                return
            except socket.error:
                logging.debug("Socket error, exiting")
                traceback.print_exc()
                return

            # Killswitch: if up button pressed, turn off motors and exit
            if buttons.up:
                SetDuty(motorDutyCycleFile_left, 0)
                SetDuty(motorDutyCycleFile_right, 0)
                logging.debug("avg_diff: %f, var_diff: %f, sampling_time: %f", avg_diff, diff_variance, p.SAMPLING_TIME)
                logging.info("Control loop stopped.")
                leds.set_color(leds.LEFT, leds.RED)
                leds.set_color(leds.RIGHT, leds.RED)
                if log_enabled:
                    f.close()
                exit()

            # If time's up, break reception loop
            if time.perf_counter() >= (next_trigger_time-0.006):
                if receptionStarted:
                    if prediction_count < 10:
                        tasrx_k = time.perf_counter()  # threoretical reception timestamp
                        if prediction_count > 0:
                            logging.debug("Prediction step %d, %d", prediction_count, k)
                        voltage_left = voltagePredictions[2*prediction_count]
                        voltage_right = voltagePredictions[2*prediction_count+1]
                        SetDuty(motorDutyCycleFile_left, voltage_left)
                        SetDuty(motorDutyCycleFile_right, voltage_right)
                        taw_k = time.perf_counter() # theoretical application timestamp
                        motor_voltage_applied_left = voltage_left
                        motor_voltage_applied_right = voltage_right
                        prediction_count += 1
                        seq_no_applied += 1
                        outdated_warning_printed = False
                    else:
                        if outdated_warning_printed == False:
                            logging.debug("Warning: No more simulation steps available!")
                            outdated_warning_printed = True
                        else:
                            pass
                # Robot logging
                if log_enabled:
                    f.write("%f,%f,%f,%f,%f\n" % (gyro, enc_l, enc_r, tau_RTT, prediction_count-1))
                break


if __name__ == "__main__":

    # Assign description to the help doc
    parser = argparse.ArgumentParser(description='Sending sensor data from EV3 and actuating data to EV3')

    # Add arguments
    parser.add_argument('-a', '--address', type=str, help='IP address of controller', required=True)
    parser.add_argument('-sp', '--sport', type=int, help='Sensor port number', default=34543)
    parser.add_argument('-ap', '--aport', type=int, help='Actuator port number', default=34544)
    parser.add_argument('-v', '--verbose', action='store_true', help='Put app in verbose mode')
    parser.add_argument('-l', '--logging', action='store_true', help='Enable logging')

    # Array for all arguments passed to script
    args = parser.parse_args()

    logger = logging.getLogger()
    if args.verbose:
        logger.setLevel(logging.DEBUG)
    else:
        logger.setLevel(logging.INFO)

    logging.debug("IP address of the controller: %s", args.address)

    #  Nice to process for better timing performance (needs sudo!)
    os.nice(-11)

    main(p.SAMPLING_TIME, args.address, args.sport, args.aport, args.logging)

