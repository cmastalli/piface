import numpy as np
import pinocchio as pin


class CenterOfPressure:
    def __init__(self, robot_wrap):
        self.robot = robot_wrap

    def compute(self, wrench_lf, wrench_rf, method='FootWrenches'):
        if method == 'FootWrenches':
            cop = self.computeThroughFootWrenches(wrench_lf, wrench_rf)
        elif method == 'FootCoPs':
            cop = self.computeThroughFootCoPs(wrench_lf, wrench_rf)
        else:
            cop = self.computeThroughFootWrenches(wrench_lf, wrench_rf)
        return cop

    def computeThroughFootCoPs(self, wrench_lf, wrench_rf):
        self.wrench_LF = wrench_lf
        self.wrench_RF = wrench_rf
        cop_lf = self.computeCoPFromWrench(self.wrench_LF)
        cop_rf = self.computeCoPFromWrench(self.wrench_RF)
        
        # Expressing the local CoP position into the world frame
        w_X_lf = self.robot.data.oMi[self.robot.legs['lf_foot']]
        w_X_rf = self.robot.data.oMi[self.robot.legs['rf_foot']]
        cop_lf_W = w_X_lf * cop_lf
        cop_rf_W = w_X_rf * cop_rf
        
        self.wrench_LF_W = w_X_lf.act(pin.Force(wrench_lf[:3], wrench_lf[3:])).vector
        self.wrench_RF_W = w_X_lf.act(pin.Force(wrench_rf[:3], wrench_rf[3:])).vector
        
        # Computing the CoP of the system, i.e.
        # p = (f^lf_z * p^lf + f^rf_z * p^rf) / (f^lf_z + f^rf_z)
        fz_lf = np.asscalar(self.wrench_LF_W[2])
        fz_rf = np.asscalar(self.wrench_RF_W[2])
        self.cop = (fz_lf * cop_lf_W + fz_rf * cop_rf_W) / (fz_lf + fz_rf)
        return self.cop

    def computeThroughFootWrenches(self, wrench_lf, wrench_rf):
        w_X_lf = self.robot.data.oMi[self.robot.legs['lf_foot']]
        w_X_rf = self.robot.data.oMi[self.robot.legs['rf_foot']]
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
