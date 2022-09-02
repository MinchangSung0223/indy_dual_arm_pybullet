import sys
from pyScurveGenerator import *
sys.path.append('/home/sung/workspace/pyScurveGenerator/example')
from draw_functions import *


traj = Trajectory();
traj.so = 0;
traj.vo = 10;
traj.ao = 0;
traj.sf = 1;
traj.vf = 18;
traj.af = 0;
traj.vmax = 342;
traj.amax = 2750;
traj.dmax = 2750;
traj.j = 30000;


sg = ScurveGenerator([traj])
sg.syncTime();

sg.printAllTrajectory()
traj = sg.getTraj(0)
j = traj.j
vo = traj.vo
vf = traj.vf
af = traj.af
ao = traj.ao
so = traj.so
sf = traj.sf
a1 = traj.a1;
a2 = traj.a2;



sg = ScurveGenerator([traj])
val = vo*vo/2/a1 +vf*vf/2/a2;

sa1 = (6*a1**2*ao**2 + 12*a1**2*j*vo + 8*a1*ao**3 + 24*a1*ao*j*vo + 3*ao**4 + 12*ao**2*j*vo + 12*j**2*vo**2)/(24*a1*j**2)
sa2 = (6*a2**2*af**2 + 12*a2**2*j*vf - 8*a2*af**3 - 24*a2*af*j*vf + 3*af**4 + 12*af**2*j*vf + 12*j**2*vf**2)/(24*a2*j**2)
print("val : ",sa1+sa2)





drawTrajList(sg,[traj],10000);
