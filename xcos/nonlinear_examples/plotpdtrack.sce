// plotpdtrack.sce  plot PD tracking performance
figure(1);
subplot(211),
plot(Q1_d.time,Q1_d.values,'k:',pd_q1tp1000d100.time,pd_q1tp1000d100.values,'b-.',pd_q1tp3000d100.time,pd_q1tp3000d100.values,'m-',pd_q1tp3000d300.time,pd_q1tp3000d300.values,'r--');
ylabel('q1 (deg)')
legend('Trajectory command','Kp = 1000I, Kd = 100I','Kp = 3000I, Kd = 100I','Kp=3000I, Kd = 300I',4);
subplot(212),
plot(Q2_d.time,Q2_d.values,'k:',pd_q2tp1000d100.time,pd_q2tp1000d100.values,'b-.',pd_q2tp3000d100.time,pd_q2tp3000d100.values,'m-',pd_q2tp3000d300.time,pd_q2tp3000d300.values,'r--');
xlabel('time (sec)')
ylabel('q2 (deg)')
legend('Trajectory command','Kp = 1000I, Kd = 100I','Kp = 3000I, Kd = 100I','Kp=3000I, Kd = 300I');
