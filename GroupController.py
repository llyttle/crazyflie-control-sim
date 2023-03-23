'''
Good PD Values without Disturbance
python3 main.py kp=5 ki=0 kd=75 sim_time=10 disturbance_flag=True

Good PID Values with Disturbance
python3 main.py kp=100 ki=1 kd=150 sim_time=10 disturbance_flag=True

'''

import utils

class Controller1D():
    """
    This class computes the commanded thrusts (N) to be applied to the quadrotor plant.

    You are to implement the "compute_commands" method.
    """
    def __init__(self, cfparams, pid_gains):
        """
        Inputs:
        - cfparams (CrazyflieParams dataclass):     model parameter class for the crazyflie
        - pid_gains (PIDGains dataclass):           pid gain class
        """
        self.params = cfparams

        # control gains
        self.kp_z = pid_gains.kp
        self.ki_z = pid_gains.ki
        self.kd_z = pid_gains.kd
        self.params = utils.CrazyflieParams
        self.E_last = 0
        self.E_accum = 0

    def compute_commands(self, setpoint, state):
        """
        Inputs:
        - setpoint (State dataclass):   the desired control setpoint
        - state (State dataclass):      the current state of the system

        Returns:
        - U (float): total upward thrust
        """

        # t_elap
        E = setpoint.z_pos - state.z_pos
        E_dot = (E-self.E_last)
        self.E_last = E
        self.E_accum += E


        U = 0 # force for upward thrusts
        # PD
        # z_accel = self.kd_z*E_dot+self.kp_z*E
        #PID 
        z_accel = self.kd_z*E_dot + self.kp_z*E + self.ki_z*self.E_accum
        U = self.params.mass*(z_accel+self.params.g)

        return U
