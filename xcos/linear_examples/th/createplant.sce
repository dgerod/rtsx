// CREATEPLANT.SCE
// Varodom Toochinda, December 2012
//
xdel(winsid());
ka1 = 0.5; ka2 = 1000; km1 = 0.1471; km2 = 0.26; ke = 0.1996; jm = 0.000156; bm = 0.001;  
kr = 1; tau = 0.0044; wn=2*%pi*30; z = 0.7; r2d = 180/%pi;
s=poly(0,'s');
// real plant
// real plant

pxv = syslin('c',ka1*ka2*km1*km2/(tau*s+1+ka2*km1));
pqxnum = (tau*s+1+ka2*km1)^2*((1-jm*wn^2)*s^2+2*z*wn*s+wn^2);
pqdotxden = ((jm*s+bm)*(tau*s+1+ka2*km1)^2+(tau*s+1)*km1*km2*ke)*(s^2+2*z*wn*s+wn^2);
pqxden = (s+0.0001)*pqdotxden;
pqdotx = syslin('c',pqxnum/pqdotxden);
Pqdotv = pqdotx*pxv;
pqx = syslin('c',r2d*pqxnum/pqxden);
Pqv = pqx*pxv;

pd = pqx;
// plant model obtained from experiment. Parameter errors are random within 10% of real
// values

ka1_a = ka1 + 0.1*ka1*(rand()-0.5);
ka2_a = ka2 + 0.1*ka2*(rand()-0.5);
km1_a = km1 + 0.1*km1*(rand()-0.5);
km2_a = km2 + 0.1*km2*(rand()-0.5);
ke_a = ke + 0.1*ke*(rand()-0.5);
jm_a = jm + 0.1*jm*(rand()-0.5);
bm_a = bm + 0.1*bm*(rand()-0.5);
tau_a = tau + 0.1*tau*(rand()-0.5);
wn_a = wn + 0.2*wn*(rand()-0.5);

pxv_a = syslin('c',ka1_a*ka2_a*km1_a*km2_a/(tau_a*s+1+ka2_a*km1_a));
pqxnum_a = (tau_a*s+1+ka2_a*km1_a)^2*((1-jm_a*wn_a^2)*s^2+2*z*wn_a*s+wn_a^2);
pqdotxden_a = ((jm_a*s+bm_a)*(tau_a*s+1+ka2_a*km1_a)^2+(tau_a*s+1)*km1_a*km2_a*ke_a)*(s^2+2*z*wn_a*s+wn_a^2);
pqxden_a = (s+0.001)*pqdotxden_a;
pqx_a = syslin('c',r2d*pqxnum_a/pqxden_a);

pqdotx_a = syslin('c',pqxnum_a/pqdotxden_a);
Pqdotv_a = pqdotx_a*pxv_a;  // t.f from input to velocity output
Pqv_a = pqx_a*pxv_a;        // t.f. from input to angle output

pdv=pqdotx_a;

Pint = syslin('c',r2d/(s+0.001));  // replace integrator with LF pole

// compute disturbance attenuation performance
Skv = syslin('c',Akv,Bkv,Ckv,Dkv);
Sk = syslin('c',Ak,Bk,Ck,Dk);
Lv = Skv*Pqdotv;
Sv=1/(1+Lv);
Tv=1-Sv;
Tv=ss2tf(Tv);
P2=Tv*Pint;
 
Sk=ss2tf(Sk);
 
Sv=ss2tf(Sv);
 
L=Sk*P2;
 
S=1/(1+L);
Td=pqdotx*S*Pint*Sv; 
Pd=pqx;
gainplot(Pd);
gainplot(Td);



