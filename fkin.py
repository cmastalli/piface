import pinocchio as pin
import numpy as np


class fkin:
    def __init__(self, robot):
        self.robot = robot
        self.pos = dict()
        self.vel = dict()
        self.acc = dict()
    
    def computePosition(self, q):
        pin.framesKinematics(self.robot.model, self.robot.data, q)
    
        frame_id = self.robot.model.getFrameId('lf_foot')
        self.pos['lf_foot'] = self.getFramePosition(frame_id).translation

        frame_id = self.robot.model.getFrameId('lh_foot')
        self.pos['lh_foot'] = self.getFramePosition(frame_id).translation

        frame_id = self.robot.model.getFrameId('rf_foot')
        self.pos['rf_foot'] = self.getFramePosition(frame_id).translation

        frame_id = self.robot.model.getFrameId('rh_foot')
        self.pos['rh_foot'] = self.getFramePosition(frame_id).translation

        return self.pos
        
    def computeVelocity(self, q, qd):
        pin.forwardKinematics(self.robot.model, self.robot.data, q, qd)
        
        frame_id = self.robot.model.getFrameId('lf_foot')
        self.vel['lf_foot'] = self.getFrameVelocity(frame_id).linear

        frame_id = self.robot.model.getFrameId('lh_foot')
        self.vel['lh_foot'] = self.getFrameVelocity(frame_id).linear

        frame_id = self.robot.model.getFrameId('rf_foot')
        self.vel['rf_foot'] = self.getFrameVelocity(frame_id).linear

        frame_id = self.robot.model.getFrameId('rh_foot')
        self.vel['rh_foot'] = self.getFrameVelocity(frame_id).linear
        
        return self.vel

    def computeAcceleration(self, q, qd, qdd):
        pin.forwardKinematics(self.robot.model, self.robot.data, q, qd, qdd)
        
        frame_id = self.robot.model.getFrameId('lf_foot')
        self.acc['lf_foot'] = self.getFrameAcceleration(frame_id).linear

        frame_id = self.robot.model.getFrameId('lh_foot')
        self.acc['lh_foot'] = self.getFrameAcceleration(frame_id).linear

        frame_id = self.robot.model.getFrameId('rf_foot')
        self.acc['rf_foot'] = self.getFrameAcceleration(frame_id).linear

        frame_id = self.robot.model.getFrameId('rh_foot')
        self.acc['rh_foot'] = self.getFrameAcceleration(frame_id).linear
        
        return self.acc
    
    def getFramePosition(self, frame_id):
        f = self.robot.model.frames[frame_id]
        return self.robot.data.oMi[f.parent].act(f.placement)
    
    def getFrameVelocity(self, frame_id):
        f = self.robot.model.frames[frame_id]
        return f.placement.actInv(self.robot.data.v[f.parent])

    def getFrameAcceleration(self, frame_id):
        f = self.robot.model.frames[frame_id]
        return f.placement.actInv(self.robot.data.a[f.parent])


np.set_printoptions(suppress=True)
from hyq import HyQ
hyq = HyQ()

q = hyq.q0
qd = pin.utils.zero(hyq.model.nv)
qdd = pin.utils.zero(hyq.model.nv)

fk = fkin(hyq)
print fk.computePosition(q)
print fk.computeVelocity(q, qd)
print fk.computeAcceleration(q, qd, qdd)