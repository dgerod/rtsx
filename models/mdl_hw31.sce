// mdl_hw31.sce  Model of Homework 3 prob.1 from my robotics course
// www.controlsystemslab.com   July 2012

clear L;
a3 = 1;
d2 = 1;  // this is default value, since d2 is a variable
L(1)= Link([0 0 0 pi/2]);
L(2)=Link([pi d2 0 pi/2],'P');    // link 2 is prismatic
L(3)=Link([0 0 a3 0]);
hw31_robot=SerialLink(L);
hw31_robot.viewangle = [45 -65];
q0 = [3*pi/4 1 pi/4];