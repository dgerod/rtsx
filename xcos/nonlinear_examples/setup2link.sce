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
