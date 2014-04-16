//plotinvdim.sce
// see how mass errors affect performance

figure(1);
subplot(211),
plot(Q1_d.time,Q1_d.values,'k:',invd_q1m08m.time,invd_q1m08m.values,'g-', invd_q1m02.time,invd_q1m02.values,'b-.',invd_q1m04.time,invd_q1m04.values,'r--');
ylabel('q1 (deg)')

subplot(212),
plot(Q2_d.time,Q2_d.values,'k:',invd_q2m08m.time,invd_q2m08m.values,'g-', invd_q2m02.time,invd_q2m02.values,'b-.',invd_q2m04.time,invd_q2m04.values,'r--');
xlabel('time (sec)')
//legend('Trajectory command','50%','80%');
ylabel('q2 (deg)')

