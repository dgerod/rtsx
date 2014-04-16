// plotadaptive.sce
// plot tracking responses from adaptive control
figure(1);
subplot(321)
plot(t,q1_d,'k:',q1.time,q1.values,'b-');
ylabel('q1');
title('Joint 1');
legend('command','response',2);
subplot(322)
plot(t,q2_d,'k:',q2.time,q2.values,'b-');
ylabel('q2');
title('Joint 2');
legend('command','response',2);
subplot(323)
plot(t,qd1_d,'k:',qd1.time,qd1.values,'b-');
ylabel('qd1');
xlabel('time (sec)');
legend('command','response',2);
ha = gca();
ha.data_bounds(1,2)= -20;
ha.data_bounds(2,2)=20;
subplot(324)
plot(t,qd2_d,'k:',qd2.time,qd2.values,'b-');
ylabel('qd2');
xlabel('time (sec)');
legend('command','response',2);
ha = gca();
ha.data_bounds(1,2)= -20;
ha.data_bounds(2,2)=20;
subplot(325)
plot(t,qdd1_d,'k:',qdd1.time,qdd1.values,'b-');
ylabel('qdd1');

legend('command','response',2);
ha = gca();
ha.data_bounds(1,2)= -100;
ha.data_bounds(2,2)=100;
subplot(326)
plot(t,qdd2_d,'k:',qdd2.time,qdd2.values,'b-');
ylabel('qdd2');

legend('command','response',2);
ha = gca();
ha.data_bounds(1,2)= -100;
ha.data_bounds(2,2)=100;
