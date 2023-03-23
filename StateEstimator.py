import numpy as np

from utils import State

class StateEstimator1D():
    """
    This class estimates the state (position and velocity) of the system based 
    on noisy sensor measurements using a kalman filter

    You are to implement the "compute" method.
    """
    def __init__(self, params, init_state):
        """
        Inputs:
        - params (CrazyflieParams dataclass):       model parameter class for the crazyflie
        - init_state (State dataclass):             initial state of the system
        """
        self.params = params

        # Define initial conditions
        self.init = init_state

        self.std2 = 0**2

        # confidence distributions
        self.process_std2 = 0.4**2
        self.sensor_std2 = 2.0**2

    def compute(self, z_meas, U, time_delta):
        """
        Estimates the current system state given the noisy measurement and control input

        Inputs:
        - z_meas (float):       current noisy measurement from the system sensor
        - U (float):            computed control input (thrust)
        - time_delta (float):   discrete time interval for simulation

        Returns:
        - filtered_state (State): estimated system state (z_pos, z_vel) using the kalman filter

        """
        _state = State()
        filtered_state = State()

        # Predict step
        _state.z_vel = (U/self.params.mass)/time_delta # Incorrect according to Kene
        _state.z_pos = self.init.z_pos + self.init.z_vel*time_delta

        _std2 = self.std2 + self.process_std2

        # Update step
        K = _std2 / (_std2 + self.sensor_std2)        # Kalman gain

        y = z_meas - _state.z_pos                   # Residual y

        filtered_state.z_pos = _state.z_pos + K*y
        self.std2 = (_std2*self.sensor_std2) / (_std2+self.sensor_std2)

        self.init = filtered_state
        print("State: ", filtered_state.z_pos)
        print("Variance: ", np.sqrt(self.std2))

        print("Kalman Velocity: ", _state.z_vel)

        return filtered_state
