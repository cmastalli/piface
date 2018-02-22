import numpy as np
import pinocchio as pin


class HyQ:
  def __init__(self):
    path = ''
    urdf = path + 'hyq.urdf'
    self.robot = pin.robot_wrapper.RobotWrapper(urdf, [path,], pin.JointModelFreeFlyer())
    self.model = self.robot.model
    self.data = self.robot.data

    # Define the nominal position
    self.q0 = np.zeros((19, 1))
    self.q0[6] = 1.
#    self.q0[7+0] = -0.2
    self.q0[7+1] = 0.75
    self.q0[7+2] = -1.5
#    self.q0[7+3] = -0.2
    self.q0[7+4] = -0.75
    self.q0[7+5] = 1.5
#    self.q0[7+6] = -0.2
    self.q0[7+7] = 0.75
    self.q0[7+8] = -1.5
#    self.q0[7+9] = -0.2
    self.q0[7+10] = -0.75
    self.q0[7+11] = 1.5


    # Create a dictionary of feet
    self.legs = dict()
    self.legs['lh_foot'] = self.robot.model.getJointId('lh_kfe_joint')
    self.legs['lf_foot'] = self.robot.model.getJointId('lf_kfe_joint')
    self.legs['rh_foot'] = self.robot.model.getJointId('rh_kfe_joint')
    self.legs['rf_foot'] = self.robot.model.getJointId('rf_kfe_joint')


hyq = HyQ()
