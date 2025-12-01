import math


class IKHandler:
    meow_s = 0.4325
    meow_e = 0.522
    meow_nigga = 0.0935

    l_s = math.sqrt(meow_s**2 + meow_nigga**2)
    l_e = meow_e
    alpha = math.atan(meow_nigga / meow_s)

    Kp = 75

    MAX_OUTPUT = 5

    def __init__(self):
        self.encoder_history = []

    def IK(self, px, py):
        r = (px ** 2 + py ** 2) ** 0.5
        theta = math.atan2(py, px)

        # angle from origin-end to origin-joint
        phi = math.acos(
            (self.l_s ** 2 + r ** 2 - self.l_e ** 2) / (2 * self.l_s * r))
        # always +ve (coz we always want the joint above the origin-end line)

        q_s = theta + phi + self.alpha

        # angle from joint-end to origin-joint
        q_e = math.pi - math.acos((self.l_s ** 2 + self.l_e **
                                  2 - r ** 2) / (2 * self.l_s * self.l_e)) + self.alpha

        return q_s, q_e

    def K(self, q_s, q_e):
        px = self.l_s * math.cos(q_s - self.alpha) + \
            self.l_e * math.cos(q_s - q_e)
        py = self.l_s * math.sin(q_s - self.alpha) + \
            self.l_e * math.sin(q_s - q_e)

        return px, py

    def PIDs(self, q_s, q_e, a_s, a_e):
        """
        :param q_s: ideal shoulder angle
        :param q_e: ideal elbow angle
        :param a_s: real shoulder angle
        :param a_e: real elbow angle
        :return:
        """

        Q = [q_s, q_e]
        A = [a_s, a_e]

        to_return = []

        for q, a in zip(Q, A):
            e = q - a

            output = e * self.Kp

            rev = output < 0

            output = abs(output)

            if output > self.MAX_OUTPUT:
                output = self.MAX_OUTPUT

            to_return.append((int(output * 51), rev))

        return tuple(to_return)
