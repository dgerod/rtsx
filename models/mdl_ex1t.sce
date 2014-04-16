//mdl_ex1t.sce  Robot model example 1 with tool orientation added
// This file creates an RPR robot 
// www.controlsystemslab.com   July 2012

clear L;
// length and offset parameters
d1 = 1;
d2 = 2;  // maximum stretch for variable d2
a3 = 1;
L(1)=Link([0 d1 0 -pi/2]);
L(2)=Link([-pi/2 d2 0 -pi/2],'P');    // prismatic joint
L(3)=Link([0 0 a3 0]);
ex1t_robot=SerialLink(L);
T = troty(pi/2);
ex1t_robot.tool = T;
ex1t_robot.name = 'Robot Arm Example 1 (with tool orientation)';
ex1t_robot.viewangle = [58.5 46.5];
clear L;