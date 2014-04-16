// mdl_myrobot.sce  make an arbitrary robot
// www.controlsystemslab.com  July 2012
// format L(i) = Link([theta d a alpha],'joint_type',);
L(1)=Link([0 2 1 %pi/4]);
L(2)=Link([0 1 2 %pi/4]);
L(3)=Link([0 1 0 0],'P');
myrobot=SerialLink(L);
