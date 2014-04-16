//plotinvdtrack.sce

figure(1);
subplot(211),
plot(Q1_d.time,Q1_d.values,'k:',invd_q1p300d20.time,invd_q1p300d20.values,'r-');
ylabel('q1 (deg)')
legend('Trajectory command','Kp1 = 300, Kd1 = 20',4);
subplot(212),
plot(Q2_d.time,Q2_d.values,'k:',invd_q2p300d20.time,invd_q2p300d20.values,'r-', invd_q2p800d70.time,invd_q2p800d70.values,'b--');
xlabel('time (sec)')
ylabel('q2 (deg)')
legend('Trajectory command','Kp2 = 300, Kd2 = 20','Kp2 = 800, Kd2 = 70');

// plot errors and compute rms values
q1d = Q1_d.values(11:501);
q2d = Q2_d.values(11:501);
tvec = Q1_d.time(11:501);
ev11=q1d - invd_q1p300d20.values;

ev21=q2d - invd_q2p300d20.values;
ev22=q2d - invd_q2p800d70.values;
ev23=q2d - invd_q2p1200d70.values;

figure(2);
subplot(211),
plot(tvec,ev11,'r-');
xlabel('kp1 = 200, kd1 = 20');
ylabel('q1 error (deg)')
subplot(212),
plot(tvec,ev21,'r-',tvec,ev22,'b-.',tvec,ev23,'k--');
xlabel('time (sec)')
ylabel('q2 error (deg)')
legend('kp2 = 300, kd2 = 20','kp2 = 800, kd2 = 70','kp2 = 1200, kd2 = 70');


datasize = size(ev11,1);
e_11 = 0;
e_21 = 0;
e_22 = 0;
e_23 = 0;
for k=1:datasize,
    e_11 = e_11+ev11(k)^2;

    e_21 = e_21+ev21(k)^2;
    e_22 = e_22+ev22(k)^2;
    e_23 = e_23+ev23(k)^2;

end
erms_11 =sqrt(e_11/datasize);

erms_21 =sqrt(e_21/datasize);
erms_22 =sqrt(e_22/datasize);
erms_23 =sqrt(e_23/datasize);

