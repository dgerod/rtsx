//testibvs4.sce  
// test IBVS4 function

//cam = CentralCamera('default');
//P = mkgrid(2, 0.5, 'T', transl(0,0,3));
//pStar = [ 312   312   712   712;
//   312   712   712   312];
//Tc0 = transl(1,1,-3)*trotz(0.6);
////[pmat,pt,vmat,emat,Tcmat,jcond,zvec]=ibvs4(cam,'T0',Tc0, 'P', P,'pstar',pStar,'niter',100,'theta',2.8,'smoothing',0.7,'depthest');
//
//[pmat,pt,vmat,emat,Tcmat,jcond,zvec]=ibvs4(cam,'T0',Tc0, 'P', P,'pstar',pStar,'niter',100,'depthest');
//
////[pmat,pt,vmat,emat,Tcmat,jcond,zvec]=ibvs4(cam,'T0',Tc0, 'P', P,'pstar',pStar,'niter',100,'depth',1);
//
////ibvs4(cam,'T0',Tc0, 'P', P,'pstar',pStar,'depth',10);
//

// Z-rotation test
cam = CentralCamera('default');
P = mkgrid(2, 0.5,'T',transl(0,0,2));

pStar = camproject(cam,P,'Tcam',trotz(0.9*pi));

ibvs4(cam, 'P', P,'pstar',pStar,'depth',2);

