// mdl_twolink.sce   make a two-link manipulator
// www.controlsystemslab.com   July 2012
clear L;
a1 = 1.2;
a2 = 1;
L(1)=Link([0 0 a1 0]);
L(2)=Link([0 0 a2 0]);
twolink=SerialLink(L);
clear L;