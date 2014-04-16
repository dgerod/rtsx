// setup2link_d.sce  
// assign values to parameters of 2-link manipulator
// use deterministic parameter errors
// www.controlsystemslab.com    Feb 2013

I1n = 2; I2n = 2;  // nominal values
m1n = 5; m2n = 5;


l1 = 1; 
l2 = 1;
m1 = m1n;  
m2 = m2n;
lc1 = l1/2; 
lc2 = l2/2;

I1 = I1n;
I2 = I2n;
g = 9.81;

d22=m2*lc2^2+I2;

// acquired plant model has parameters within +/- 10% of real plant
//et=1.2;

m1_e = m1n + et*m1n; 
m2_e = m2n + et*m2n;
lc1_e = lc1; // + et*lc1;
lc2_e = lc2; // + et*lc2;
I1_e = I1n; + et*I1n;
I2_e = I2n; + et*I2n;


d22_e=m2_e*lc2_e^2+I2_e;

// for adaptive control
lamda = 60;
//kth=10;
kth=100;
kth1 = kth;
kth2 = kth;
kth3 = kth;
kth4 = kth;
kth5 = kth;
