import time
from typing import List, Tuple


class VelocityController:
    def __init__(self, Kp = 100000, Ki = 0, Kd = 0, I_time = 0.5, D_time = 0.01, acc_coef = 1.0):
        """
        :param Kp: P parameter of PID
        :param Ki: I parameter of PID
        :param Kd: D parameter of PID
        :param I_time: Time parameter for integral
        :param D_time: Time parameter for derivative
        :param acc_coef: Scaling coefficient for acceleration (equivalent to scaling PID parameters by same amount)
        """
        self.Kp = Kp
        self.Ki = Ki
        self.Kd = Kd
        self.I_time = I_time
        self.D_time = D_time
        self.acc_coef = acc_coef

        self.desired_velocity = 0
        self.real_velocity = 0
        self.current_output = 0
        self.position_history: List[Tuple[float, float]] = []
        self.error_history: List[Tuple[float, float]] = []

    def tick(self):
        """
        Call this whenever anything changes
        :return:
        """
        self.update_error()

        if len(self.error_history) < 2:
            return

        pid_val = self.get_PID()

        self.set_output(pid_val)

        self.publish()

    def update_error(self):
        error = self.desired_velocity - self.real_velocity
        self.error_history.append((time.perf_counter(), error))
        if len(self.error_history) > 1_000_000:
            self.error_history = self.error_history[-10_000:]

    def get_PID(self) -> float:
        """
        :return: returns PID value based on errors and parameters
        """
        P_term = self.error_history[-1][1]

        I_term = 0
        prev_t = time.perf_counter()
        for t, e in self.error_history[::-1]:
            if time.perf_counter() - t > self.I_time:
                break

            I_term += e * (prev_t - t)
            prev_t = t

        other_t = time.perf_counter()
        other_e = self.error_history[-1][1]
        for other_t, other_e in self.error_history[-2::-1]:
            if (time.perf_counter() - other_t) > self.D_time:
                break
        D_term = (self.error_history[-1][1] - other_e) / (time.perf_counter() - other_t)


        return self.Kp * P_term + self.Ki * I_term + self.Kd * D_term

    def set_output(self, pid_val: float):
        self.current_output += self.acc_coef * pid_val * (time.perf_counter() - self.error_history[-2][0])

    def publish(self):
        # publish self.current_output
        pass

    def set_velocity(self, v: float):
        self.desired_velocity = v
        self.tick()

    def add_position(self, pos: int | float):
        self.position_history.append((time.perf_counter(), pos))
        if len(self.position_history) > 1_000_000:
            self.position_history = self.position_history[-10_000:]
        self.update_real_velocity()
        self.tick()

    def update_real_velocity(self):
        if len(self.position_history) < 2:
            self.real_velocity = 0
        else:
            self.real_velocity = ((self.position_history[-1][1] - self.position_history[-2][1]) /
                                  (self.position_history[-1][0] - self.position_history[-2][0]))
