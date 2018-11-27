import controller_params as p


class Filter:
    """
    Class that computes the state vector out of the sensor measurements
    """
    def __init__(self, gyro_offset=0, r=0.0275, w=0.087):

        # Bias of the gyroscope
        self.gyro_offset = gyro_offset
        self.start_gyro_bias_filter = False
        # Wheel radius
        self.r = r
        # Length of the axis between the wheels
        self.w = w

        #  Pitch angle [rad]
        self.theta0 = -1.0325 # Initial pitch angle [rad]
        self.theta = self.theta0

        #  Pitch rate [rad/s]
        self.theta_dot0 = 0.0
        self.theta_dot = self.theta_dot0

        #  Wheel angle [rad]
        self.phi0 = 0.0
        self.phi = self.phi0

        #  Wheel rate [rad/s]
        self.phi_dot0 = 0.0
        self.phi_dot = self.phi_dot0

        #  Integral of the wheel angle [rad^2]
        self.phi_int0 = 0.0
        self.phi_int = self.phi_int0

        #  Yaw angle [rad]
        self.gamma0 = 0.0
        self.gamma = self.gamma0

        #  Yaw rate [rad/s]
        self.gamma_dot0 = 0.0
        self.gamma_dot = self.gamma_dot0

    def init_gyro_offset(self, gyro_offset):
        """
        Initializes the gyro offset
        :param gyro_offset: Measured gyroscope bias
        :return:
        """
        self.gyro_offset = gyro_offset

    def update_gyro_offset(self, gyro_rate):
        """
        Updates the gyro offset to compensate the integral error
        :param gyro_rate: Measured gyro rate
        :return:
        """
        self.gyro_offset = (1 - 0.0015) * self.gyro_offset + 0.0015 * gyro_rate   #GEAENDERT!

    def calculate_states(self, gyro_rate, enc_l, enc_r, ts):
        """
        It calculates the state vector out of the measured values.

        :param gyro_rate: Measured gyro rate coming from the 1D gyroscope
        :param enc_l: Measured incremental left wheel angle
        :param enc_r: Measured incremental right wheel angle
        :return: state vector
        """

        self.theta_dot = (gyro_rate - self.gyro_offset) * p.rad_per_deg
        self.theta = self.theta + self.theta_dot * ts
        if self.theta > 0:
            self.start_gyro_bias_filter = True
        phi_old = self.phi
        self.phi = p.rad_per_deg * (enc_l + enc_r) / 2 + self.theta

        self.phi_dot = (self.phi - phi_old) / ts

        self.phi_int = self.phi_int + self.phi * ts

        gamma_old = self.gamma
        self.gamma = p.rad_per_deg * (enc_r - enc_l) * self.r / self.w

        self.gamma_dot = (self.gamma - gamma_old) / ts
        if self.start_gyro_bias_filter:
            self.update_gyro_offset(gyro_rate)

        return self.phi_int,\
               self.phi, self.theta, \
               self.phi_dot, self.theta_dot, \
               self.gamma, self.gamma_dot

               
               
