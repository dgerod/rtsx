// plotpd.sce  plot PD step responses
figure(1);
subplot(211),
plot(step1.time,step1.values,'k:',pd_q1sp1000.time,pd_q1sp1000.values,'b-',pd_q1sp2000.time,pd_q1sp2000.values,'m-',pd_q1sp3000.time,pd_q1sp3000.values,'r-');
ylabel('q1 (deg)')
subplot(212),
plot(step2.time,step2.values,'k:',pd_q2sp1000.time,pd_q2sp1000.values,'b-',pd_q2sp2000.time,pd_q2sp2000.values,'m-',pd_q2sp3000.time,pd_q2sp3000.values,'r-');
xlabel('time (sec)')
ylabel('q2 (deg)')

figure(2);
subplot(211),
plot(step1.time,step1.values,'k:',pd_q1sd150.time,pd_q1sd150.values,'b-',pd_q1sd250.time,pd_q1sd250.values,'m-',pd_q1sd350.time,pd_q1sd350.values,'r-');
ylabel('q1 (deg)')
subplot(212),
plot(step2.time,step2.values,'k:',pd_q2sd150.time,pd_q2sd150.values,'b-',pd_q2sd250.time,pd_q2sd250.values,'m-',pd_q2sd350.time,pd_q2sd350.values,'r-');
xlabel('time (sec)')
ylabel('q2 (deg)')
