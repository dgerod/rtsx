// terror.sce  compute and plot tracking error

et = [];
for k=1:size(q.values,1)
    et=[et; Qt.values(k) - q.values(k)];
end
figure(1),plot(q.time,et);
xlabel('time (sec)');
ylabel('tracking error (deg)');
