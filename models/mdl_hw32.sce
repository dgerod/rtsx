// mdl_hw32.sce  Model of Homework 3 prob.2 from my robotics course
// www.controlsystemslab.com   July 2012

clear L;
d1 = 1;
a2 = 1;
a3 = 1;
L(1)= Link([0 d1 0 pi/2]);
L(2)=Link([0 0 a2 0]);    
L(3)=Link([0 0 a3 0]);
hw32_robot=SerialLink(L);
hw32_robot.viewangle = [65, -55];
q0 = [0 0 0];
clear L;