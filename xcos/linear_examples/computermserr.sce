// computermserr.sce
// compute RMS errors

//load('track.dat');
datasize = size(hinf1_err,1);
e_pid = 0;
e_hinf1 = 0;
e_hinf2 = 0;
for k=1:datasize,
    e_pid = e_pid+pid_err(k)^2;
    e_hinf1 = e_hinf1+hinf1_err(k)^2;
    e_hinf2 = e_hinf2+hinf2_err(k)^2;
end
erms_pid = sqrt(e_pid/datasize);
erms_hinf1 = sqrt(e_hinf1/datasize);
erms_hinf2 = sqrt(e_hinf2/datasize);
