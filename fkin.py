import pinocchio as pin
import numpy as np


class fkin:
  def __init__(self, robot):
    self.robot = robot
    
  def compute(self):
    pin.forwardKinematics(self.robot.model, self.robot.data, self.robot.q0)
    
    frame_id = self.robot.model.getFrameId('lf_foot')
    print 'lf_foot:', self.robot.robot.framePosition(self.robot.q0, frame_id, False).translation.T

    frame_id = self.robot.model.getFrameId('lh_foot')
    print 'lh_foot:', self.robot.robot.framePosition(self.robot.q0, frame_id, False).translation.T

    frame_id = self.robot.model.getFrameId('rf_foot')
    print 'rf_foot:', self.robot.robot.framePosition(self.robot.q0, frame_id, False).translation.T

    frame_id = self.robot.model.getFrameId('rh_foot')
    print 'rh_foot:', self.robot.robot.framePosition(self.robot.q0, frame_id, False).translation.T
    f = self.robot.model.frames[frame_id]
    print '        ', self.robot.data.oMi[f.parent].act(f.placement).translation.T



np.set_printoptions(suppress=True)
from hyq import HyQ
hyq = HyQ()

fk = fkin(hyq)
fk.compute()
