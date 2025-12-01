import math
from typing import List


class VroomVroom:
    def __init__(self):
        self.a = 0.5
        self.b = 0.2
        self.speed_scalar = 1
        self.deadzone = 0.1

    def smooooth_operatorrrr(self, joy_x: float, joy_y: float) -> List[List[float]]:
        """
        Smooth Operator mode: The rover moves on a circular path with Radius of curvature and velocity according to joystick input
        :param joy_x: X axis of Joystick (from -1 to 1)
        :param joy_y: Y axis of Joystick (from -1 to 1)
        :return: Returns 2 lists of angles and velocities respectively. The order of motors is in: Front Right, Back Right, Back Left, Front Right
        """

        if abs(joy_x) <= self.deadzone:
            # go straight
            if abs(joy_y) <= self.deadzone:
                vel = 0
            else:
                vel = joy_y

            return [
                [0] * 4,
                [vel] * 4
            ]

        angles = [0.0] * 4
        vels = [0.0] * 4

        rinv = math.tan(joy_x * (math.pi / 2 - 0.01))

        R = 1 / rinv

        if abs(joy_y) <= self.deadzone:
            w = 0
        else:
            w = self.speed_scalar * (joy_y / (1 + abs(R))) * (1 if R > 0 else -1)

        t1 = math.atan2(self.b, R - self.a)
        if R < 0:
            t1 -= math.pi

        R1 = (self.b ** 2 + (R - self.a) ** 2) ** 0.5
        if R < 0:
            R1 *= -1

        angles[0] = -t1
        angles[1] = t1

        vels[0] = w * R1
        vels[1] = w * R1

        t2 = math.atan2(self.b, R + self.a)
        if R < 0:
            t2 -= math.pi

        R2 = (self.b ** 2 + (R + self.a) ** 2) ** 0.5
        if R < 0:
            R2 *= -1

        angles[2] = -t2
        angles[3] = t2

        vels[2] = w * R2
        vels[3] = w * R2

        return [angles, vels]


v = VroomVroom()

print(v.smooooth_operatorrrr(0, 0.9))
print(v.smooooth_operatorrrr(0.5, 0.9))
print(v.smooooth_operatorrrr(0.9, 0))
print(v.smooooth_operatorrrr(-0.6, 0.9))