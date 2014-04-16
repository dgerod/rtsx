// setup2link.sce  
// assign values to parameters of 2-link manipulator
// www.controlsystemslab.com    Feb 2013

l1 = 1; l2 = 1;
m1 = 5; m2 = 5;
lc1 = l1/2; lc2 = l2/2;
I1 = 2; I2 = 2;
g = 9.81;

d22=m2*lc2^2+I2;

// acquired plant model has parameters within +/- 10% of real plant
et = 1.0;

m1_e = m1 + et*m1*(rand()-0.5);
m2_e = m2 + et*m2*(rand()-0.5);
lc1_e = lc1 + et*lc1*(rand()-0.5);
lc2_e = lc2 + et*lc2*(rand()-0.5);
I1_e = I1 + et*I1*(rand()-0.5);
I2_e = I2 + et*I2*(rand()-0.5);

d22_e=m2_e*lc2_e^2+I2_e;

// for adaptive control
lamda = 60; //60;
kth=100;
//kth=100;
kth1 = kth;
kth2 = kth;
kth3 = kth;
kth4 = kth;
kth5 = kth;
