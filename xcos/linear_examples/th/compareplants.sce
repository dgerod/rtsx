//compareplants.sce 
xdel(winsid());
ka1 = 0.5; ka2 = 1000; km1 = 0.1471; km2 = 0.26; ke = 0.1996; jm = 0.000156; bm = 0.001;  
kr = 1; tau = 0.0044; wn=2*%pi*30; z = 0.7; r2d = 180/%pi;
s=poly(0,'s');
// real plant

rnum = wn^2;
rden = s^2+2*z*wn*s+wn^2;
pr = syslin('c',rnum/rden);
px = syslin('c',km1*km2*ke*(tau*s+1)/(tau*s+1+ka2*km1)^2);
pj = syslin('c',1/(jm*s+bm));
p1 = syslin('c',jm*s/(jm*s+bm));
p2 = 1/(1+pj*px);
p3 = pj/(1+pj*px);
pint = syslin('c',1/(s+0.0001));
pqx1 = r2d*(p3 - p2*p1*pr*s)*pint;

pxv = syslin('c',ka1*ka2*km1*km2/(tau*s+1+ka2*km1));

Pqv1 = pqx1*pxv;

// method 2
pqxnum = (tau*s+1+ka2*km1)^2*((1-jm*wn^2)*s^2+2*z*wn*s+wn^2);
pqxden = (s+0.0001)*((jm*s+bm)*(tau*s+1+ka2*km1)^2+(tau*s+1)*km1*km2*ke)*(s^2+2*z*wn*s+wn^2);
pqx2 = syslin('c',r2d*pqxnum/pqxden);
Pqv2 = pqx2*pxv;

// compare frequency responses
[frq,pqures]=repfreq(Pqv1,0.01,1000);
[frq1,pqures_a]=repfreq(Pqv2, frq);
frq=frq';
fsize = size(frq,1);
pqumag = zeros(fsize,1);
pquph = zeros(fsize,1);
pqumag_a = zeros(fsize,1);
pquph_a = zeros(fsize,1);

for k=1:fsize,
    [pqumag(k),pquph(k)]=polar(pqures(k));
    [pqumag_a(k),pquph_a(k)]=polar(pqures_a(k));    
end
pquph = pquph*180/%pi;
pquph_a = pquph_a*180/%pi;
figure(1);
subplot(211),plot2d("ln",frq,[20*log10(pqumag) 20*log10(abs(pqumag_a))]);
ylabel('Magnitude (dB)');
subplot(212),plot2d("ln",frq,[20*log10(abs(pquph)) 20*log10(abs(pquph_a))]);
ylabel('Phase (degree)');
xlabel('Frequency (Hz)');








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


[A,B,C,D]=abcd(Pqv2);
//[Ap,Bp,Cp,Dp]=abcd(Pqu_a);
[Ad,Bd,Cd,Dd]=abcd(pqx2);
