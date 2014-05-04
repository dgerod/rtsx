//mdl_sphwrist.sce  spherical wrist
// This file creates a spherical wrist 
// www.controlsystemslab.com   October 2012

clear L;
// length and offset parameters
d1 = 1;
d3 = 1;  

L(1)=Link([0 d1 0 pi/2]);
L(2)=Link([0 0 0 -pi/2]);    
L(3)=Link([0 d3 0 0]);
sph_wrist=SerialLink(L);
sph_wrist.name = 'Spherical Wrist';
sph_wrist.viewangle = [58.5 46.5];
q0 = [0 0 0];
clear L;
