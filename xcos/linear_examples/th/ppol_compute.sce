//ppol_compute.sce  pole placement for 1-joint robot

// plant state space data
s=poly(0,'s');
P=1/(s^2+s);
Pss = tf2ss(P);

// pole placement
z=0.7; wn=18;
//z=0.7; wn = 40;
lamda = s^2+2*z*wn*s+wn^2;
clpoles = roots(lamda);

K = ppol(Pss.A,Pss.B,clpoles);
