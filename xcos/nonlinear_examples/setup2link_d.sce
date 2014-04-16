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

// trajectory command generation
t=0:0.01:5;
[q1_d,qd1_d,qdd1_d]=lspb(0,30,t);
[q2_d,qd2_d,qdd2_d]=lspb(0,-10,t);
Q1_d.time = t'; Q1_d.values = q1_d;
Qd1_d.time = t'; Qd1_d.values = qd1_d;
Qdd1_d.time = t'; Qdd1_d.values = qdd1_d;
Q2_d.time = t'; Q2_d.values = q2_d;
Qd2_d.time = t'; Qd2_d.values = qd2_d;
Qdd2_d.time = t'; Qdd2_d.values = qdd2_d;
