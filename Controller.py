import time
import utils

class Controller1D_pretest():
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
        # self.kp_z = pid_gains.kp
        # self.ki_z = pid_gains.ki
        # self.kd_z = pid_gains.kd

        # self.kp_z = 20.0          # 3.512s Second settle
        # self.ki_z = 15.0          # 'smashing stick'
        # self.kd_z = 0.0           # .02m steady state error

        # self.kp_z = 4.0             # 5.970s settle
        # self.ki_z = 5.5             # 'steady stick'
        # self.kd_z = 0.3             # .000m steady state error

        # self.kp_z = 5.0             # 4.164s settle
        # self.ki_z = 5.5             # 'smashing stick'
        # self.kd_z = 0.3             # .004m steady state error

        # Cubed error tests =======================================
        self.kp_z = 400.0           # 1.505s Second settle
        self.ki_z = 480.0           # 'smooth stick'
        self.kd_z = 0.6              # -.002m steady state error

        # self.kp_z = 40.0           # 1.505s Second settle
        # self.ki_z = 60.0           # 'smooth stick'
        # self.kd_z = 0.6              # -.002m steady state error

        self.last_error = 0.0
        self.last_I = 0.0
        self.last_Pos = 0.0
        self.last_Vel = 0.0
        self.last_t = time.time()

    def compute_commands(self, setpoint, state):
        """
        Inputs:
        - setpoint (State dataclass):   the desired control setpoint
        - state (State dataclass):      the current state of the system

        Returns:
        - U (float): total upward thrust ACCELERATION
        """
        U = 0.0

        t = time.time()
        elapsed = float(t-self.last_t)

        error = setpoint.z_pos - state.z_pos

        error = error**3

        P = self.kp_z * error
        I = self.ki_z * error*elapsed + self.last_I
        D = self.kd_z * (error - self.last_error)/elapsed

        Pos = P + I + D
        Vel = (Pos - self.last_Pos)/elapsed
        Acc = (Vel - self.last_Vel)/elapsed

        # Updating stored variables
        self.last_error = error
        self.last_I = I
        self.last_Pos = Pos
        self.last_Vel = Vel
        self.last_t = t

        U = Vel + (0.03 * 9.81)
        # U = Acc + (0.030 * 9.81)

        if U > .7:
            U = .7
        elif U < 0:
            U = 0

        print("|P: {:0.3f}|".format(P), "|I: {:0.3f}|".format(I), "|D: {:0.3f}|".format(D))
        print("U: ", U)

        return U

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
        # self.kp_z = pid_gains.kp
        # self.ki_z = pid_gains.ki
        # self.kd_z = pid_gains.kd

        self.kp_z = 2.0
        self.ki_z = 0.0
        self.kd_z = 12.0

        self.last_error = 0.0
        self.last_I = 0.0
        self.last_t = time.time()

    def compute_commands(self, setpoint, state):
        """
        Inputs:
        - setpoint (State dataclass):   the desired control setpoint
        - state (State dataclass):      the current state of the system

        Returns:
        - U (float): total upward thrust ACCELERATION
        """
        U = 0.0

        t = time.time()
        elapsed = float(t-self.last_t)

        error = setpoint.z_pos - state.z_pos

        P = self.kp_z * error
        I = self.ki_z * error*elapsed + self.last_I
        D = self.kd_z * (error - self.last_error) / elapsed

        # Updating stored variables
        self.last_error = error
        self.last_I = I
        self.last_t = t

        U = self.params.mass * (P + I + D + self.params.g)

        if U > .7:
            U = .7
        elif U < 0:
            U = 0

        print("|P: {:0.3f}|".format(P), "|I: {:0.3f}|".format(I), "|D: {:0.3f}|".format(D))
        print("U: ", U)

        return U