from hrp2 import HRP2
import numpy as np
import pinocchio as pin

class CenterOfPressure:
    def __init__(self, robot_wrap):
        self.robot = robot_wrap
        

    def computeCoP(self, wrench_lf, wrench_rf):
        self.wrench_LF = wrench_lf
        self.wrench_RF = wrench_rf
        cop_lf = self.computeCoPFromWrench(self.wrench_LF)
        cop_rf = self.computeCoPFromWrench(self.wrench_RF)
        
        # Expressing the local CoP position into the world frame
        w_X_lf = self.robot.data.oMi[self.robot.bodyToIdx['lf']]
        w_X_rf = self.robot.data.oMi[self.robot.bodyToIdx['rf']]
        cop_lf_W = w_X_lf * cop_lf
        cop_rf_W = w_X_rf * cop_rf
        
        self.wrench_LF_W = w_X_lf.act(pin.Force(wrench_lf[:3], wrench_lf[3:])).vector
        self.wrench_RF_W = w_X_lf.act(pin.Force(wrench_rf[:3], wrench_rf[3:])).vector
        
        # Computing the CoP of the system, i.e.
        # p = (f^lf_z * p^lf + f^rf_z * p^rf) / (f^lf_z + f^rf_z)
        fz_lf = np.asscalar(self.wrench_LF_W[2])
        fz_rf = np.asscalar(self.wrench_RF_W[2])
        cop = (fz_lf * cop_lf_W + fz_rf * cop_rf_W) / (fz_lf + fz_rf)
        return cop
    
    def computeCoP2(self, wrench_lf, wrench_rf):
        w_X_lf = self.robot.data.oMi[cop.robot.bodyToIdx['lf']]
        w_X_rf = self.robot.data.oMi[cop.robot.bodyToIdx['rf']]
        self.wrench_lf_W = w_X_lf.act(pin.Force(wrench_lf[:3], wrench_lf[3:]))
        self.wrench_rf_W = w_X_rf.act(pin.Force(wrench_rf[:3], wrench_rf[3:]))
        self.wrench_T = (self.wrench_lf_W + self.wrench_rf_W).vector
        
        return self.computeCoPFromWrench(self.wrench_T)
        
    def computeCoPFromWrench(self, wrench):
        cop = np.matrix([0., 0., 0.]).T
#        self.FOOT_FORCE_SENSOR_XYZ = np.matrix([0.0,   0.0, -0.085]).T
#        sole_X_ftSens = pin.SE3(np.eye(3), -self.FOOT_FORCE_SENSOR_XYZ);
        sole_X_ftSens = pin.SE3(np.eye(3), np.matrix([0., 0., 0.]).T);
        wrench = pin.Force(wrench[:3], wrench[3:]);
        wrench = sole_X_ftSens.act(wrench);
        
        # The CoP is defined as the point where the pressure force moments 
        # vanishes. This occurs when tau = [p]x f, which the 6d force (i.e. 
        # wrench) is [f,tau]. It also equivalents to: p = [n]x tau / (f.n),
        # which simplifies is p = [-tau_y, tau_x] / f_z
        cop[0] = -wrench.angular[1] / wrench.linear[2];
        cop[1] =  wrench.angular[0] / wrench.linear[2];

        return cop
        

import pickle
from dynamic_graph.sot.torque_control.hrp2.sot_utils import config_sot_to_urdf

# Getting the data to analyses
filename = '/home/cmastall/data/iparams-id/cd1.txt'
data = pickle.load(open(filename))

for d in data:
    q = config_sot_to_urdf(np.asmatrix(d.q))
    wrench_lf = d.f_lf
    wrench_rf = d.f_rf


    # Evaluation of the center of pressure from different methods
    hrp2 = HRP2()
    hrp2.update(q)
    cop = CenterOfPressure(hrp2)

    cop_pos = cop.computeCoP(wrench_lf, wrench_rf)
    print cop_pos.T
    print cop.computeCoP2(wrench_lf, wrench_rf).T
    print '---'