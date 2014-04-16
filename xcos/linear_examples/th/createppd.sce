//createppd.sce 
xdel(winsid());
ka1 = 0.5; ka2 = 1000; km1 = 0.1471; km2 = 0.26; ke = 0.1996; jm = 0.000156; bm = 0.001;  
kr = 1; tau = 0.0044; wn=2*%pi*30; z = 0.7; r2d = 180/%pi;
s=poly(0,'s');
// real plant

resnum = wn^2;
resden = s^2+2*z*wn*s+wn^2;
pr = syslin('c',resnum/resden);
pf = syslin('c',km1*km2*ke*(tau*s+1)/(tau*s+1+ka2*km1)^2);
pj = syslin('c',1/(jm*s+bm));
pd1 = syslin('c',jm*s/(jm*s+bm));
pd2 = 1/(1+pj*pf);
pd3 = pj/(1+pj*pf);
pint = syslin('c',1/(s+0.0001));
pd = r2d*(pd3 - pd2*pd1*pr*s)*pint;

ptv = syslin('c',ka1*ka2*km1*km2/(tau*s+1+ka2*km1));

Pqv = ptv*pd;








// ******************** old ***************

ka1_a = ka1 + 0.1*ka1*(rand()-0.5);
ka2_a = ka2 + 0.1*ka2*(rand()-0.5);
km1_a = km1 + 0.1*km1*(rand()-0.5);
km2_a = km2 + 0.1*km2*(rand()-0.5);
ke_a = ke + 0.1*ke*(rand()-0.5);
jm_a = jm + 0.1*jm*(rand()-0.5);
bm_a = bm + 0.1*bm*(rand()-0.5);
tau_a = tau + 0.1*tau*(rand()-0.5);
wn_a = wn + 0.2*wn*(rand()-0.5);


[A,B,C,D]=abcd(Pqv);
//[Ap,Bp,Cp,Dp]=abcd(Pqu_a);
[Ad,Bd,Cd,Dd]=abcd(pd);
