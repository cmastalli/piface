import rospkg
import pinocchio as pin
from pinocchio.robot_wrapper import RobotWrapper
import numpy as np


class HRP2:
    def __init__(self):
        # Finding the urdf file
        self.rospack = rospkg.RosPack()
        self.path = self.rospack.get_path('hrp2_14_description')
        self.filename = "hrp2_14.urdf"
        self.urdf = self.path + "/urdf/" + self.filename
        
        # Parsing the urdf in Pinocchio and adding the free flyer (i.e. floating-base)
        self.robot = RobotWrapper(self.urdf, [self.path,], pin.JointModelFreeFlyer())
        self.model = self.robot.model
        self.data = self.model.createData()
        
         # Create a dictionary of feet
        self.legs = dict()
        self.legs['lf_foot'] = self.robot.model.getJointId('LLEG_JOINT5')
        self.legs['rf_foot'] = self.robot.model.getJointId('RLEG_JOINT5')
        self.K = [4034., 23770., 239018., 707., 502., 936.]
        
    def update(self, q):
        pin.forwardKinematics(self.model, self.data, q)
        pin.computeAllTerms(self.model, self.data, q, np.asmatrix(np.zeros(self.robot.nv)).T)

    def compute_sMa(self, wrench):
        self.rpy = -np.matrix([np.asscalar(wrench[3] / self.K[3]),
                               np.asscalar(wrench[4] / self.K[4]),
                               np.asscalar(wrench[5] / self.K[5])]).T
        self.R = pin.utils.rpyToMatrix(self.rpy)
        self.t = -np.matrix([np.asscalar(wrench[0] / self.K[0]),
                             np.asscalar(wrench[1] / self.K[1]),
                             np.asscalar(wrench[2] / self.K[2])]).T
        return pin.SE3(self.R, self.t)