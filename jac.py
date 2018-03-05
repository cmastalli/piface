import pinocchio as pin
import numpy as np


class jac:
  def __init__(self, robot):
    self.robot = robot
    
  def compute(self):
    frame_id = self.robot.model.getFrameId('lh_foot')
    print pin.frameJacobian(self.robot.model, self.robot.data, frame_id, self.robot.q0)
    print '---'
    print pin.computeJacobians(self.robot.model, self.robot.data, self.robot.q0)
    print '---'
    print pin.se3.jacobian(self.robot.model, self.robot.data, self.robot.q0, frame_id, False, True) #False == world


np.set_printoptions(suppress=True)
from robots.hyq import HyQ
hyq = HyQ()

j = jac(hyq)
j.compute()
