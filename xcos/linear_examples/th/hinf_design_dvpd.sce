// HINF_DESIGN_DV.SCE   perform H-inf design on servo robot joint   
// disturbance attenuation on velocity loop
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
pqxden = (s+0.0001)*((jm*s+bm)*(tau*s+1+ka2*km1)^2+(tau*s+1)*km1*km2*ke)*(s^2+2*z*wn*s+wn^2);
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

// control design starts here
// performance weighting function on S
a=0.0003; m=2; wb=200;
w1_n=((1/m^0.5)*s+wb)^2;
w1_d=(s+wb*a^0.5)^2;
w1 = syslin('c',w1_n,w1_d);

// weighting on KS
w2_d=2*(s/3000 + 1);
w2_n=s/500+1;
w2 = syslin('c',w2_n,w2_d);

// plot weighting functions
figure(2);
subplot(211), gainplot(1/w1);
gainplot(1/pdv);
ylabel('magnitude (dB)');
title('1/w1');
subplot(212), gainplot(1/w2);
ylabel('magnitude (dB)');
title('1/w2');

//return;
// form generalized plant
//N = [wp_n, wp_n*Pqunum_a; 0, wt_n*Pqunum_a; 1, Pqunum_a];
//D = [wp_d, wp_d*Pquden_a; 1, wt_d*Pquden_a; 1, Pquden_a];
//Pgen=syslin('c',N,D);
[Ap1,Bp1,Cp1,Dp1]=abcd(pxv_a);
[Ap2,Bp2,Cp2,Dp2]=abcd(pqdotx_a);
[Aw1,Bw1,Cw1,Dw1]=abcd(w1);
[Aw2,Bw2,Cw2,Dw2]=abcd(w2);

ap1_r=size(Ap1,1); ap1_c = size(Ap1,2);
ap2_r=size(Ap2,1); ap2_c = size(Ap2,2);
aw1_r=size(Aw1,1); aw1_c = size(Aw1,2);
aw2_r=size(Aw2,1); aw2_c = size(Aw2,2);

bp1_r=size(Bp1,1);
bp2_r=size(Bp2,1);
bw1_r=size(Bw1,1);
bw2_r=size(Bw2,1);

cp1_c = size(Cp1,2);
cp2_c = size(Cp2,2);
cw1_c = size(Cw1,2);
cw2_c = size(Cw2,2);

Agp1 = [Ap1 zeros(ap1_r,ap2_c) zeros(ap1_r,aw1_c) zeros(ap1_r,aw2_c);
     Bp2*Cp1 Ap2 zeros(ap2_r,aw1_c) zeros(ap2_r,aw2_c);
     zeros(aw1_r,ap1_c) Bw1*Cp2 Aw1 zeros(aw1_r,aw2_c);
     zeros(aw2_r,ap1_c) zeros(aw2_r,ap2_c) zeros(aw2_r,aw1_c) Aw2];
Bgp1 = [zeros(bp1_r,1) -Bp1;
        Bp2 zeros(bp2_r,1);
        zeros(bw1_r,1) zeros(bw1_r,1);
        zeros(bw2_r,1) -Bw2];
Cgp1 = [zeros(1,cp1_c) Dw1*Cp2 Cw1 zeros(1,cw2_c);
        zeros(1,cp1_c) zeros(1,cp2_c) zeros(1,cw1_c) Cw2;
        zeros(1,cp1_c) Cp2 zeros(1,cw1_c) zeros(1,cw2_c)];
Dgp1 = [0 0.001; 0 -Dw2;0.001 0];
  // makes D12 full rank


Pgen1=syslin('c',Agp1,Bgp1,Cgp1,Dgp1);

// use h_inf function
[Skv,ro]=h_inf(Pgen1, [1,1],0, 200000, 100);
[Akv,Bkv,Ckv,Dkv]=abcd(Skv);

//use hinf function
//[Akv,Bkv,Ckv,Dkv]=hinf(Agp1,Bgp1,Cgp1,Dgp1,1,1,10);
//Skv=syslin('c',Akv,Bkv,Ckv,Dkv);
// form closed-loop system and check stability

Lv_a = Skv*Pqdotv_a;
Sv_a=1/(1+Lv_a);
Tv_a = 1 - Sv_a;
figure(3)
gainplot(Sv_a);
figure(4)
gainplot(Tv_a);
[Aclv_a,Bclv_a,Cclv_a,Dclv_a]=abcd(Tv_a);



figure(5)
gainplot(Sv_a*pdv);
//[Ak,Bk,Ck,Dk] = hinf(Ap,Bp,Cp,Dp,1,1,10);

