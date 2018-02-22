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
        
        self.bodyToIdx = dict()
        self.bodyToIdx['lf'] = self.robot.index('LLEG_JOINT5')
        self.bodyToIdx['rf'] = self.robot.index('RLEG_JOINT5')
        
    def update(self, q):
        pin.forwardKinematics(self.model, self.data, q)
        pin.computeAllTerms(self.model, self.data, q, np.asmatrix(np.zeros(self.robot.nv)).T)
