"""
Controller for a cyber physical network based on Lego Mindstorms.
"""
import datetime
import argparse
import logging
import time

import numpy as np
from scipy import io

import packet
import controller_filter as f
import controller_socket as cs

import controller_params as p

class Controller:
    """
    Extending a model with control functionality.
    """
    rad_per_deg = 3.14159 / 180

    # Threshold angle at which the controller kicks in
    start_threshold = 0.0

    def __init__(self, ip_address, aport, sport, measurement_folder):
        self.ref_vec =np.array([[0], [0], [0], [0], [0], [0], [0]])
        self.controller_socket = cs.ControllerSocket(ip_address, aport, sport, measurement_folder)
        self.filter = f.Filter()

    def control_loop(self):
        """
        Control loop method. Receives measurements,
        calculates control inputs array and sends packet to robot.
        """

        # Inits
        ts = p.SAMPLING_TIME
        k = 0
        first_packet = True
        # Load matrices for system simulation from MATLAB file
        dictionary = io.loadmat('controller_simulation_matrices')
        A_d30 = dictionary['Ad_30']
        B_d30 = dictionary['Bd_30']
        A_d35 = dictionary['Ad_35']
        B_d35 = dictionary['Bd_35']
        logging.debug("Packet format: %s", packet.H2R_PACKET_FORMAT)

        seq_number = 0
        seq_number_last = 0
        start_actuation = False
        first_packet = True
        logging.info("Controller started. Waitig for packets from the Robot.")
        while True:
            data = self.controller_socket.receive()
            if data:
                if first_packet:
                    logging.info("Control loop started.")
                    first_packet = False

                logging.debug("Received["+str(k)+"]: " + str(time.perf_counter()))
                k += 1
                # Received measurement from the sensor
                seq_number = data[0]
                gyro_rate = data[1]
                enc_l = data[2]
                enc_r = data[3]
                gyro_offset = data[4]
                motorVoltageApplied_left = data[5]
                motorVoltageApplied_right = data[6]

                if seq_number-seq_number_last > 1:
                    logging.debug("Packet lost")
                seq_number_last = seq_number
                if first_packet is True:
                    self.filter.init_gyro_offset(gyro_offset)
                    first_packet = False

                # Filter states calculation from received measurements
                phi_int, phi, theta, phi_dot, theta_dot, gamma, gamma_dot = self.filter.calculate_states(gyro_rate, enc_l, enc_r, ts)

                # Create state and reference array
                x0 = np.array([[phi_int], [phi], [theta], [phi_dot], [theta_dot], [gamma], [gamma_dot]])

                # Calculate voltage list
                voltage_list = self.delay_compensation_controller(A_d30, B_d30, A_d35, B_d35, x0, self.ref_vec,
                                                                  motorVoltageApplied_left,
                                                                  motorVoltageApplied_right)
                # If theta smaller than starting threshold, send control input
                if theta > self.start_threshold:
                    # High threshold so the controller stays in action from now on
                    start_actuation = True
                # Transmission of motor voltage to the Robot
                if start_actuation:
                    self.controller_socket.send(seq_number, *voltage_list)
                    logging.debug("C: Sent actuation %d", seq_number)
                else:
                    logging.debug("Controller waiting to kick in: %f", theta)

    def calc_control(self, ref_vec, state_vec, k):
        """
        Calculates the motor voltages.

        :param ref_vec:   numpy array with reference states
        :param state_vec: numpy array with states
        :param k:         control params.

        :return:          motor_voltage_left/_right, left/right motor voltage
        """
        k2 = np.copy(k)
        k2[5:7] = -k2[5:7]
        motor_voltage_left = np.matmul(k, ref_vec-state_vec)
        motor_voltage_right = np.matmul(k2, ref_vec-state_vec)
        return motor_voltage_left[0], motor_voltage_right[0]

    def simulateStates(self, A_discreteSys, B_discreteSys, x0, sim_steps, motorVoltageApplied_left,
                       motorVoltageApplied_right):
        """
        Method to simulate a linear, discrete time system representing the robot.

        :param A_discreteSys:             state space matrix
        :param B_discreteSys:             state space matrix
        :param x0:                        inital states
        :param sim_steps:                 number of steps to be simulated
        :param motorVoltageApplied_left:  voltage applied at the moment of the measurement at the robot
        :param motorVoltageApplied_right: voltage applied at the moment of the measurement at the robot
        :return: xout:                    numpy array containing the simulation. Each row represents a
                                          state, each column a simulation time step
        """
        # Input array
        u = np.array([[motorVoltageApplied_left], [motorVoltageApplied_right]])
        # Preallocate output array
        xout = np.zeros((7, sim_steps))
        xlast = x0
        # Simulation:
        for i in range(sim_steps):
            xout[:, i, None] = np.matmul(A_discreteSys, xlast) + np.matmul(B_discreteSys, u)
        return xout

    def delay_compensation_controller(self, A_d30, B_d30, A_d35, B_d35, x0,
                                      ref_vec, motorVoltageApplied_left, motorVoltageApplied_right):
        """
        Method that simulates the robot's state trajectories for different delays. For each delay the corresponding
        control input is calculated. Additionally, the robots state trajectories are simulated for the case that the
        voltages calculated before (for each delay) are being applied for one sample period. Those simulations are
        used to calculate control inputs, that can be used at the robots side, if packet loss occurs.

        :param A_discreteSys:             state space matrix
        :param B_discreteSys:             state space matrix
        :param x0:                        inital states array
        :param ref_vec:                   reference states array
        :param motorVoltageApplied_left:  voltage applied at the moment of measurement at the robot
        :param motorVoltageApplied_right: voltage applied at the moment of measurement at the robot
        :return: voltage_list:            list containing control inputs+predictions for each delay
        """

        # LQR control params for 35ms sample time
        k1 = -0.161394
        k2 = -0.427475
        k3 = -56.843234
        k4 = -1.080444
        k5 = -9.007788
        k6 = -6.140744
        k7 = -0.045586
        k = np.array([k1, k2, k3, k4, k5, k6, k7])
        # State space matrices for 8ms sample time
        no_of_delays = 1
        no_of_sim_steps = 10
        # Init voltage list
        no_of_voltages_per_delay = 2*(1+no_of_sim_steps)
        voltage_list = [0]*(no_of_delays*no_of_voltages_per_delay)
        # Simulate delays
        xout = self.simulateStates(A_d30, B_d30, x0, 1,
                                   motorVoltageApplied_left, motorVoltageApplied_right)
        # Calculate control input+
        voltage_left, voltage_right = self.calc_control(ref_vec,
                                                        x0,
                                                        k)
        # Clip values to max voltage
        if abs(voltage_left) > 8:
            logging.debug("SAT: %f"%voltage_left)
            voltage_left = np.sign(voltage_left)*8
        if abs(voltage_right) > 8:
            voltage_right = np.sign(voltage_right)*8
        # Store in voltage list
        voltage_list[0] = int(voltage_left*1000000)
        voltage_list[1] = int(voltage_right*1000000)
        for i in range(no_of_sim_steps):
            # Simulate system for one sample period, using above values as input and simulated states for current
            # delay as starting point
            xout = self.simulateStates(A_d35, B_d35,
                                       xout[:, -1, None],
                                       1, voltage_left, voltage_right)
            # Calc predicted control input 1
            voltage_left, voltage_right = self.calc_control(ref_vec, xout[:, -1, None], k)
            # Clip values:
            if abs(voltage_left) > 8:
                voltage_left = np.sign(voltage_left)*8
            if abs(voltage_right) > 8:
                voltage_right = np.sign(voltage_right)*8
            # Store in list
            voltage_list[2*i+2] = int(voltage_left*1000000)
            voltage_list[2*i+3] = int(voltage_right*1000000)
        return voltage_list

if __name__ == "__main__":

    # Assign description to the help doc
    parser = argparse.ArgumentParser(description='Distributed controller application')

    # Add arguments
    parser.add_argument('-a', '--address', type=str, help='IP address of destination', default='0.0.0.0')
    parser.add_argument('-m', '--measurement_folder', type=str, help='Subfolder for measurements', default='.')
    parser.add_argument('-sp', '--sport', type=int, help='Sensor port number', default=34543)
    parser.add_argument('-ap', '--aport', type=int, help='Actuator port number', default=34544)
    parser.add_argument('-v', '--verbose', action='store_true', help='Put app in verbose mode')

    # Array for all arguments passed to script
    args = parser.parse_args()

    # Configure logging
    logger = logging.getLogger()
    if args.verbose:
        logger.setLevel(logging.DEBUG)
    else:
        logger.setLevel(logging.INFO)

    logging.debug("IP address of the application: %s", args.address)

    # Logging from the controller
    timenow = datetime.datetime.now()
    try:
        controller = Controller(args.address, args.aport, args.sport, args.measurement_folder)
        controller.control_loop()
    except KeyboardInterrupt:
        logging.info("Control loop stopped.")
        exit()
