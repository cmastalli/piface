import pinocchio as pin


class fkin:
  def __init__(self, robot):
    self.robot = robot
    
  def compute(self):
    pin.forwardKinematics(self.robot.model, self.robot.data, self.robot.q0)
    
    frame_id = self.robot.model.getFrameId('lf_foot')
    print 'lf_foot:', self.robot.robot.framePosition(self.robot.q0, frame_id).translation.T

    frame_id = self.robot.model.getFrameId('lh_foot')
    print 'lh_foot:', self.robot.robot.framePosition(self.robot.q0, frame_id).translation.T

    frame_id = self.robot.model.getFrameId('rf_foot')
    print 'rf_foot:', self.robot.robot.framePosition(self.robot.q0, frame_id).translation.T

    frame_id = self.robot.model.getFrameId('rh_foot')
    print 'rh_foot:', self.robot.robot.framePosition(self.robot.q0, frame_id).translation.T



from hyq import HyQ
hyq = HyQ()

fk = fkin(hyq)
fk.compute()
