// pdtrack_compute_rmserr.sce
// compute RMS errors

//load('pdtrack.dat');
q1d = Q1_d.values(11:501);
q2d = Q2_d.values(11:501);
tvec = Q1_d.time(11:501);
ev11=q1d - pd_q1tp1000d100.values;
ev12=q1d - pd_q1tp3000d100.values;
ev13=q1d - pd_q1tp3000d300.values;

ev21=q2d - pd_q2tp1000d100.values;
ev22=q2d - pd_q2tp3000d100.values;
ev23=q2d - pd_q2tp3000d300.values;

figure(1);
subplot(211),
plot(tvec,ev11,'b-.',tvec,ev12,'k-',tvec,ev13,'r:');
ylabel('q1 error (deg)')
subplot(212),
plot(tvec,ev21,'b-.',tvec,ev22,'k-',tvec,ev23,'r:');
xlabel('time (sec)')
ylabel('q2 error (deg)')
legend('Kp = 1000I, Kd = 100I','Kp = 3000I, Kd = 100I','Kp=3000I, Kd = 300I',3);


datasize = size(ev11,1);
e_11 = 0;
e_12 = 0;
e_13 = 0;
e_21 = 0;
e_22 = 0;
e_23 = 0;
for k=1:datasize,
    e_11 = e_11+ev11(k)^2;
    e_12 = e_12+ev12(k)^2;
    e_13 = e_13+ev13(k)^2;
    e_21 = e_21+ev21(k)^2;
    e_22 = e_22+ev22(k)^2;
    e_23 = e_23+ev23(k)^2;

end
erms_11 =sqrt(e_11/datasize);
erms_12 =sqrt(e_12/datasize);
erms_13 =sqrt(e_13/datasize);
erms_21 =sqrt(e_21/datasize);
erms_22 =sqrt(e_22/datasize);
erms_23 =sqrt(e_23/datasize);

