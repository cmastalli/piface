import numpy as np
import pinocchio as pin


class CenterOfMass:
    def __init__(self, robot_wrap):
        self.robot = robot_wrap
    
    def computePos(self, q):
        com = pin.centerOfMass(self.robot.model, self.robot.data, q)
        return com # self.robot.data.com[0]

    def computePosVel(self, q, v):
        pin.centerOfMass(self.robot.model, self.robot.data, q, v)
        return self.robot.data.com[0], self.robot.data.vcom[0]

    def computePosVelAcc(self, q, v, a):
        pin.centerOfMass(self.robot.model, self.robot.data, q, v, a)
        return self.robot.data.com[0], self.robot.data.vcom[0], self.robot.data.acom[0]


np.set_printoptions(suppress=True)
from hyq import HyQ
hyq = HyQ()
com = CenterOfMass(hyq)

# The robot state
q = hyq.q0
qd = pin.utils.zero(hyq.model.nv); qd[0] = 1.
qdd = pin.utils.zero(hyq.model.nv)

c = com.computePos(q)
print c.T
c, v = com.computePosVel(q, qd)
print c.T, v.T
c, v, a = com.computePosVelAcc(q, qd, qdd)
print c.T, v.T, a.T
