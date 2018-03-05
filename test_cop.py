import pickle
import numpy as np
from cop import CenterOfPressure
from robots.hrp2 import HRP2
from dynamic_graph.sot.torque_control.hrp2.sot_utils import config_sot_to_urdf

import pinocchio as pin

# Getting the data to analyses
filename = '/home/cmastall/data/iparams-id/sim.txt'
#filename = '/home/cmastall/data/iparams-id/cd1.txt'
data = pickle.load(open(filename))


hrp2 = HRP2()
#hrp2.model.inertias[1].mass = 17.15427539
#hrp2.model.inertias[1].lever = np.matrix([-0.03937377, -0.03042923,  0.20766652]).T
#hrp2.model.inertias[1].mass = 19.1460940
#hrp2.model.inertias[1].lever = np.matrix([-0.03528609, -0.02668647,  0.21055677]).T

cop = CenterOfPressure(hrp2)

px = []; py = []
cx = []; cy = []
for d in data:
    q = config_sot_to_urdf(np.asmatrix(d.q))
    wrench_lf = d.f_lf
    wrench_rf = d.f_rf

    # Evaluation of the center of pressure from different methods
    hrp2.update(q)

    p = cop.compute(wrench_lf, wrench_rf, 'FootWrenches')
    c = pin.centerOfMass(hrp2.model, hrp2.data, q)
    print 'foot wrenches', cop.compute(wrench_lf, wrench_rf, 'FootWrenches').T
    print 'foot cops    ', cop.compute(wrench_lf, wrench_rf, 'FootCoPs').T
    
    
    wrench_lf_s = hrp2.compute_sMa(wrench_lf).act(pin.Force(wrench_lf[:3],wrench_lf[3:])).vector
    wrench_rf_s = hrp2.compute_sMa(wrench_rf).act(pin.Force(wrench_rf[:3],wrench_rf[3:])).vector
    wrench_t = wrench_lf_s + wrench_rf_s
    mass = np.linalg.norm(wrench_t[:3]) / 9.81
    print 'mass         ', mass
    print '---'
    cx.append(np.asscalar(c[0]))
    cy.append(np.asscalar(c[1]))
    px.append(np.asscalar(p[0]))
    py.append(np.asscalar(p[1]))


#import pinocchio as pin
#print pin.centerOfMass(hrp2.model, hrp2.data, q).T

x = np.arange(-0.02,0.02,0.001)
y = np.arange(-0.1,0.1,0.001)

import matplotlib.pyplot as plt
import matplotlib.gridspec as gridspec

gs = gridspec.GridSpec(nrows=1, ncols=2)

fig = plt.figure(figsize=(1, 2))
ax0 = fig.add_subplot(gs[0, 0])
ax0.plot(np.vstack(cx), np.vstack(px), 'red', linewidth=2.5)
ax0.plot(x, x, 'black', linewidth=2.)
ax0.set_xlabel(r' $c_x$', {'color':'k', 'fontsize':20})
ax0.set_ylabel(r' $p_x$', {'color':'k', 'fontsize':20})
ax0.grid(True)
ax0.spines['right'].set_visible(False)
ax0.spines['top'].set_visible(False)

ax1 = fig.add_subplot(gs[0, 1])
ax1.plot(np.vstack(cy), np.vstack(py), 'red', linewidth=2.5)
ax1.plot(y, y, 'black', linewidth=1.5)
ax1.set_xlabel(r' $c_y$', {'color':'k', 'fontsize':20})
ax1.set_ylabel(r' $p_y$', {'color':'k', 'fontsize':20})
ax1.grid(True)
ax1.spines['right'].set_visible(False)
ax1.spines['top'].set_visible(False)
plt.show()
