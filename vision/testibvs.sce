// TESTIBVS.SCE  Image-based visual servo
// 
cam = CentralCamera('default');
P = mkgrid(2, 0.5, 'T', transl(0,0,3));
pStar = [ 312   312   712   712;
   312   712   712   312];

Tc0 = transl(1,1,-3)*trotz(0.6);
depth = 1;

lambda = 0.08;

p0 = CamPlot(cam, P, 'Tcam', Tc0);
p = p0;
niter = 100;
pmat = zeros(2,4,niter);  // keep point data
vmat = zeros(6,niter);
emat = zeros(4,niter);
Tcmat = zeros(4,4,niter);
jcond = zeros(1,niter);
Tc = Tc0;
printf("\nTotal iterations = %d\n",niter);
for k=1:niter   // run for niter iterations
    printf("\r%d",k);
    e = pStar - p;
    for j = 1:4  // computing error norms
        emat(j,k) = norm(e(:,j));
    end
    e = e(:);

    J = visjac_p(cam, p, depth);
    jcond(k) = cond(J);
    v = lambda*pinv(J)*e;
    vmat(:,k) = v;
    Tc = trnorm(Tc*delta2tr(v));
    p = CamProject(cam, P, 'Tcam', Tc);
    pmat(:,:,k) = p;
    Tcmat(:,:,k) = Tc;
end

// plotting 
// points
tk=1:100;
CamPlot(cam,p0,'color','black','figure',1,'size',5);
CamPlot(cam,pmat,'color','red','style','+','hold');
CamPlot(cam,pStar,'color','blue','style','o','size',5,'hold');

// errors
figure(2);
plot(tk,emat);
xlabel('time');
ylabel('errors');
legend('e1','e2','e3','e4');
title('Point errors');



// velocity
figure(3);
plot(tk,vmat);
xlabel('time');
ylabel('Cartesian velocity');
legend('Vx','Vy','Vz','Wx','Wy','Wz');
title('Camera velocity');

// pose
xyz=t2d(Tcmat);
rpy=tr2rpy(Tcmat);
figure(4);
subplot(211), plot(tk,xyz);
legend('X','Y','Z');
ylabel('Camera position');
title(' Camera pose');
subplot(212), plot(tk,rpy);
legend('R','P','Y');
xlabel('time');
ylabel('Camera orientation');

// Jacobian condition number
figure(5);
plot(tk,jcond);
xlabel('time');
ylabel('Jacobian condition number');
title('Jacobian condition number');

