// setup_adaptive_2link.sce  
// assign values to parameters of 2-link manipulator
// and adaptive controllers
// www.controlsystemslab.com    Feb 2013

l1 = 1; l2 = 1;
m1 = 5; m2 = 5;
lc1 = l1/2; lc2 = l2/2;
I1 = 2; I2 = 2;
g = 9.81;

d22=m2*lc2^2+I2;

// for adaptive control
lamda = 60; //60;
kth=100;
//kth=100;
kth1 = kth;
kth2 = kth;
kth3 = kth;
kth4 = kth;
kth5 = kth;

// command trajectory generation
t1=0:0.01:1.99;
t2 = 0:0.01:0.99;

// joint 1 trajectory
[q11_d,qd11_d,qdd11_d]=qpoly(0,10,t1,0,10);
[q12_d,qd12_d,qdd12_d]=qpoly(10,-5,t1,10,-5);
[q13_d,qd13_d,qdd13_d]=qpoly(-5,0,t2,-5,0);
q1_d = [q11_d;q12_d;q13_d];
qd1_d = [qd11_d;qd12_d;qd13_d];
qdd1_d = [qdd11_d;qdd12_d;qdd13_d];

// joint 2 trajectory
[q21_d,qd21_d,qdd21_d]=qpoly(0,-5,t2,0,-10);
[q22_d,qd22_d,qdd22_d]=qpoly(-5,10,t1,-10,10);
[q23_d,qd23_d,qdd23_d]=qpoly(10,0,t1,10,0);
q2_d = [q21_d;q22_d;q23_d];
qd2_d = [qd21_d;qd22_d;qd23_d];
qdd2_d = [qdd21_d;qdd22_d;qdd23_d];

// put in Xcos workspace
t=0:0.01:4.99;
t=t';
Q1_d.time = t; Q1_d.values = q1_d
Qd1_d.time = t; Qd1_d.values = qd1_d;
Qdd1_d.time = t; Qdd1_d.values = qdd1_d;
Q2_d.time = t; Q2_d.values = q2_d
Qd2_d.time = t; Qd2_d.values = qd2_d;
Qdd2_d.time = t; Qdd2_d.values = qdd2_d;

