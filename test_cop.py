import pickle
import numpy as np
from cop import CenterOfPressure
from hrp2 import HRP2
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

    print 'foot wrenches', cop.compute(wrench_lf, wrench_rf, 'FootWrenches').T
    print 'foot cops    ', cop.compute(wrench_lf, wrench_rf, 'FootCoPs').T
    print '---'
