//mdl_cylindrical.sce  cylindrical robot
// This file creates a cylindrical (RPP) robot 
// www.controlsystemslab.com   October 2012

clear L;
// length and offset parameters
d1 = 1;
a1 = 1;
a2 = 1;
d3 = 0.2;
d4 = 0.2;
L(1)=Link([0 d1 a1 0]);
L(2)=Link([0 0 a2 pi]);    
L(3)=Link([0 d3 0 0],'P');
L(4)=Link([0 d4 0 0]);
scara_robot=SerialLink(L);
scara_robot.name = 'SCARA Robot';
scara_robot.viewangle = [67.25 -33.5];
q0 = [0 0 1 0];
clear L;
