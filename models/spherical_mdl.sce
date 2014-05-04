//mdl_spherical.sce  spherical robot
// This file creates a spherical (RRP) robot 
// www.controlsystemslab.com   October 2012

clear L;
// length and offset parameters
d1 = 1;
d3 = 2;  // maximum stretch for variable d3

L(1)=Link([0 d1 0 pi/2]);
L(2)=Link([0 0 0 pi/2]);    
L(3)=Link([0 d3 0 0],'P');
sph_robot=SerialLink(L);
sph_robot.name = 'Spherical Robot (RRP)';
sph_robot.viewangle = [58.5 46.5];
q0 = [pi/2 pi/2 1];
clear L;
