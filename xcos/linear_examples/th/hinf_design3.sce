// HINF_DESIGN3.SCE   perform H-inf design on servo robot joint  
// take disturbance transfer function into account 
// use two weights Wp1 and Wp2
// Varodom Toochinda, December 2012
//
xdel(winsid());
ka1 = 0.5; ka2 = 1000; km1 = 0.1471; km2 = 0.26; ke = 0.1996; jm = 0.000156; bm = 0.001;  
kr = 1; tau = 0.0044; wn=2*%pi*30; z = 0.7; r2d = 180/%pi;
s=poly(0,'s');
// real plant
pxv = syslin('c',ka1*ka2*km1*km2/(tau*s+1+ka2*km1));
pqxnum = (tau*s+1+ka2*km1)^2*((1-jm*wn^2)*s^2+2*z*wn*s+wn^2);
pqxden = (s+0.0001)*((jm*s+bm)*(tau*s+1+ka2*km1)^2+(tau*s+1)*km1*km2*ke)*(s^2+2*z*wn*s+wn^2);
pqx = syslin('c',r2d*pqxnum/pqxden);
Pqv = pqx*pxv;

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
pqxden_a = (s+0.001)*((jm_a*s+bm_a)*(tau_a*s+1+ka2_a*km1_a)^2+(tau_a*s+1)*km1_a*km2_a*ke_a)*(s^2+2*z*wn_a*s+wn_a^2);
pqx_a = syslin('c',r2d*pqxnum_a/pqxden_a);
Pqv_a = pqx_a*pxv_a;

// compute frequency responses
[frq,pqures]=repfreq(Pqv,0.01,1000);
[frq1,pqures_a]=repfreq(Pqv_a, frq);
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

// compute disturbance transfer function

pd = pqx_a;



figure(2),bode(pd);
title('transfer function that disturbance sees');

// control design starts here
// performance weighting function on S
a=0.0005; m=5; wb=100;
wp1_n=(1/m)*s+wb;

wp1_d=(s+wb*a);
wp1 = syslin('c',wp1_n/wp1_d);

a2=1; m2 = 5; wb2= 100;
wp2_n=(1/m2)*s+wb2;
wp2_d=10*(s+wb2*a2);
wp2=syslin('c',wp2_n/wp2_d);
// weighting on T

wt_d=2*(s/1000 + 1);
wt_n=s/300+1;
wt = syslin('c',wt_n/wt_d);

// plot weighting functions
figure(3);
subplot(211), gainplot(1/wp);
ylabel('magnitude (dB)');
title('Weight on S');
subplot(212), gainplot(1/wt);
ylabel('magnitude (dB)');
title('Weight on T');

// form generalized plant
//N = [wp_n, wp_n*Pqunum_a; 0, wt_n*Pqunum_a; 1, Pqunum_a];
//D = [wp_d, wp_d*Pquden_a; 1, wt_d*Pquden_a; 1, Pquden_a];
//Pgen=syslin('c',N,D);

[Ap,Bp,Cp,Dp]=abcd(Pqv_a);
[Ad,Bd,Cd,Dc]=abcd(pd);
[Awp1,Bwp1,Cwp1,Dwp1]=abcd(wp1);
[Awp2,Bwp2,Cwp2,Dwp2]=abcd(wp2);
[Awt,Bwt,Cwt,Dwt]=abcd(wt);

Agp=[Ap zeros(size(Ap,1),size(Ad,2)) zeros(size(Ap,1),size(Awp1,2)) zeros(size(Ap,1),size(Awp2,2)) zeros(size(Ap,1),size(Awt,2));
    zeros(size(Ad,1),size(Ap,2)) Ad zeros(size(Ad,1),size(Awp1,2)) zeros(size(Ad,1),size(Awp2,2)) zeros(size(Ad,1),size(Awt,2));
    -Bwp1*Cp -Bwp1*Cd Awp1 zeros(size(Awp1,1),size(Awp2,2)) zeros(size(Awp1,1),size(Awt,2));
    Bwp2*Cp Bwp2*Cd zeros(size(Awp2,1),size(Awp1,2)) Awp2 zeros(size(Awp2,1),size(Awt,2));
    Bwt*Cp zeros(size(Awt,1),size(Ad,2)) zeros(size(Awt,1),size(Awp1,2)) zeros(size(Awt,1),size(Awp2,2)) Awt];
Bgp = [zeros(size(Bp,1),size(Bd,2)) zeros(size(Bp,1),size(Bwp1,2)) Bp; 
        Bd zeros(size(Bd,1),size(Bwp,2)) zeros(size(Bd,1),size(Bp,2));
        zeros(size(Bwp1,1),size(Bd,2)) Bwp1 zeros(size(Bwp1,1),size(Bp,2));
        0 0 0;
        zeros(size(Bwt,1),size(Bd,2)) zeros(size(Bwt,1),size(Bwp1,2)) 0];
Cgp = [-Dwp1*Cp -Dwp1*Cd Cwp1 zeros(size(Cwp1,1),size(Cwp2,2)) zeros(size(Cwp1,1),size(Cwt,2));
        Dwp2*Cp Dwp2*Cd zeros(size(Cwp2,1),size(Cwp1,2)) Cwp2 zeros(size(Cwp2,1),size(Cwt,2));
        zeros(size(Cwt,1),size(Cp,2)) zeros(size(Cwt,1),size(Cd,2)) zeros(size(Cwt,1),size(Cwp1,2)) zeros(size(Cwt,1),size(Cwp2,2)) Cwt;
        -Cp -Cd zeros(size(Cp,1),size(Cwp1,2)) zeros(size(Cp,1),size(Cwp2,2)) zeros(size(Cp,1),size(Cwt,2))];
Dgp = [0 Dwp1 0.0001; 0 0 0; 0 0 Dwt;0 1 0];  // makes D12 full rank


Pgen=syslin('c',Agp,Bgp,Cgp,Dgp);
//[Sk,ro]=h_inf(Pgen, [1,1],0, 20000, 100);
[Ak,Bk,Ck,Dk]=hinf(Agp,Bgp,Cgp,Dgp,1,1,10);
Sk=syslin('c',Ak,Bk,Ck,Dk);
// form closed-loop system and check stability
L=Sk*Pqv;
S=1/(1+L); T = 1 -S;
[Acl,Bcl,Ccl,Dcl]=abcd(T);
figure(4)
gainplot(S);
figure(5)
gainplot(T);
La = Sk*Pqv_a;
Sa=1/(1+La);
Ta = 1 - Sa;
[Acla,Bcla,Ccla,Dcla]=abcd(Ta);

//[Ak,Bk,Ck,Dk]=abcd(Sk);


//[Ak,Bk,Ck,Dk] = hinf(Ap,Bp,Cp,Dp,1,1,10);

