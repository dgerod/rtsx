//plotinvdie.sce
// see how inertia errors affect performance

figure(1);
subplot(211),
plot(Q1_d.time,Q1_d.values,'k:',invd_q1i06m.time,invd_q1i06m.values,'b-', invd_q1i03.time,invd_q1i03.values,'b-.',invd_q1i05.time,invd_q1i05.values,'r--');
ylabel('q1 (deg)')

subplot(212),
plot(Q2_d.time,Q2_d.values,'k:',invd_q2i06m.time,invd_q2i06m.values,'b-', invd_q2i03.time,invd_q2i03.values,'b-.',invd_q2i05.time,invd_q2i05.values,'r--');
xlabel('time (sec)')
//legend('Trajectory command','50%','80%');
ylabel('q2 (deg)')

