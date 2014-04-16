//mdl_cylindrical.sce  cylindrical robot
// This file creates a cylindrical (RPP) robot 
// www.controlsystemslab.com   October 2012

clear L;
// length and offset parameters
d1 = 1;
d2 = 2;  // maximum stretch for variable d2, d3
d3 = 1;
L(1)=Link([0 d1 0 0]);
L(2)=Link([0 d2 0 -pi/2],'P');    // prismatic joint
L(3)=Link([0 1 0 0],'P');
cylind_robot=SerialLink(L);
cylind_robot.name = 'Cylindrical Robot (RPP)';
cylind_robot.viewangle = [58.5 46.5];
q0 = [0 1 1];
clear L;
