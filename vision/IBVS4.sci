// IBVS4.SCI 
// www.controlsystemslab.com   April 2013
// Implements Image-Based Visual Servoing for 4 feature points

    // Options::
    // 'niter',N         Maximum number of iterations
    // 'eterm',E         Terminate when norm of feature error < E
    // 'lambda',L        Control gain, positive definite scalar or matrix
    // 'T0',T            The initial pose
    // 'P',p             The set of world points (3xN)

    
    // 'pstar',p         The desired image plane coordinates
    // 'depth',D         Assumed depth of points is D (default true depth
    //                   from simulation is assumed)
    // 'depthest'        Run a simple depth estimator

    // 'verbose'         Print out extra information during simulation
    //
    // Notes::
    // - If 'P' is specified it overrides the default square target.

function [pmat,pt,vmat,emat,Tcmat,jcond,zvec]=IBVS4(cam, varargin)
    [pmat,pt,vmat,emat,Tcmat,jcond,zvec]=_Ibvs4(cam, varargin);
endfunction

function [pmat,pt,vmat,emat,Tcmat,jcond,zvec]=ibvs4(cam, varargin)
    [pmat,pt,vmat,emat,Tcmat,jcond,zvec]=_Ibvs4(cam, varargin);
endfunction

function [pmat,pt,vmat,emat,Tcmat,jcond,zvec]=_Ibvs4(cam, varargin)
    xdel(winsid());
    printf("\nImage-Based Visual Servo algorithm for 4-point image features.\n");
    varargin = varargin($);
    varnum=length(varargin);
    
    opt.Tc0 = eye(4,4);
    opt.niter = 1000;    // maximum number of iteration
    
    opt.eterm = 10e-1;  // terminate when norm of feature error <E
    opt.P = [];          // set of world points
   
    opt.pstar = [];       // the desired image plane coordinates
    opt.d=1;         // depth of points
    opt.depthest = False;   // simple depth estimator
    opt.lambda = 0.08;       // control grain
    opt.verbose = False;

    // parameters used by depth_estimator
    // good result
    vs.theta = 3;  
    vs.smoothing = 1; 


    for iv =1:varnum  
         if type(varargin(iv))==10 then  // check if string 
              varargin(iv)=convstr(varargin(iv),'l'); // convert to lower 
          end
          if type (varargin(iv))==10 then
          
              select varargin(iv),

                   case 't0' then
                     if ishomog(varargin(iv+1))
                         opt.Tc0 = varargin(iv+1);
                     else error('Wrong data type for T0. Must be a 4 x 4 homogeneous matrix');
                     end
                   case 'p' then
                     if type(varargin(iv+1))==1 & size(varargin(iv+1))==[3,4]
                         opt.P = varargin(iv+1);
                     else error('Wrong data type for P. Must be a 3 x 4 world point coordinates');
                     end                             
 
                   case 'pstar' then
                     if type(varargin(iv+1))==1 & size(varargin(iv+1))==[2,4]
                         opt.pstar = varargin(iv+1);
                     else error('Wrong data type for pstar. Must be a 2 x 4 pixel coordinates');
                     end                             
                   case 'niter' then
                     if type(varargin(iv+1))==1 & length(varargin(iv+1))==1
                         opt.niter = round(varargin(iv+1));
                         
                     else error('Wrong data type for maximum iteration (must be an integer)');
                     end   
                   case 'eterm' then
                     if type(varargin(iv+1))==1 & length(varargin(iv+1))==1
                         opt.eterm = varargin(iv+1);
                     else error('Wrong data type for feature error target(must be a scalar)');
                     end    
                  case 'depth' then
                     if type(varargin(iv+1))==1 & length(varargin(iv+1))==1
                         opt.d = varargin(iv+1);
                     else error('Wrong data type for depth (must be a scalar)');
                     end                       
                   case 'lambda' then
                     if type(varargin(iv+1))==1 & length(varargin(iv+1))==1
                         opt.lambda = varargin(iv+1);
                     else error('Wrong data type for lambda (must be a scalar)');
                     end                       
                   case 'theta' then
                     if type(varargin(iv+1))==1 & length(varargin(iv+1))==1
                         vs.theta = varargin(iv+1);
                     else error('Wrong data type for theta (must be a scalar)');
                     end 
                   case 'smoothing' then
                     if type(varargin(iv+1))==1 & length(varargin(iv+1))==1
                         vs.smoothing = varargin(iv+1);
                     else error('Wrong data type for smoothing (must be a scalar)');
                     end 
                  case 'depthest' then 
                     opt.depthest = True;
                  case 'verbose' then
                      opt.verbose = True;
                                                  
                                         
               end // select varargin(iv)
          end  // if type (varargin(iv))== 10     
      end // for iv=1:varnum    
      

      p0 = CamProject(cam, opt.P, 'Tcam', opt.Tc0);
      p = p0;
      
     // data containers
     pmat = zeros(2,4,1);   // keep feature points data
     vmat = zeros(6,1);     // camera velovity
     emat = zeros(4,1);    // error norm for each point
     etmp = zeros(4,1);
     Tcmat = zeros(4,4,1);   // camera pose
     jcond = 0;             // jacobian condition number
     zvec = zeros(4,1);             // estimated z data
     Tc = opt.Tc0;
     vs.Tcam = Tc;  // used by depth estimator
     vs.v_old = 1e-4*ones(6,1);  // gives small value to prevent div-by-0 error
     vs.p_old = p;
     printf("\n -------- of maximum %d iterations",opt.niter);
     for k=1:opt.niter
         printf("\r%d",k);
         
         // estimate the depth if depthest mode is chosen
         if opt.depthest
             zest = depth_estimator(cam, vs, p);
             zest = zest(:);
             zvec = [zvec zest];
         end
         e = opt.pstar - p;
         
         for j=1:4  // computing error norms
             etmp(j) = norm(e(:,j));
         end
         emat = [emat etmp];  
         e = e(:);   // make a single column of 8 elements
         if opt.depthest
             J = visjac_p(cam, p, zest); 
         else    
             J = visjac_p(cam, p, opt.d);
         end
         jcond = [jcond cond(J)];
         
         v = opt.lambda*pinv(J)*e;
         vs.v_old = v;  // used by depth estimator
         vmat = [vmat v];
         Tc = trnorm(Tc*delta2tr(v));
         vs.Tcam = Tc;  // used by depth estimator
         vs.p_old = p;  // used by depth estimator
         p = CamProject(cam, opt.P, 'Tcam', Tc);
         pmat(:,:,k) = p;
         Tcmat(:,:,k) = Tc;
         if max(etmp)<opt.eterm  // error less than tolerance
             break;
         end
    end
    emat = emat(:,2:size(emat,2));  // truncate the first zero column
    vmat = vmat(:,2:size(vmat,2));
    jcond = jcond(1,2:size(jcond,2));
    
   
    // plotting 
    // points
    tk=1:k;
    CamPlot(cam,p0,'color','black','figure',1,'size',5);
    //CamPlot(cam,pmat,'color','red','style','+','size',5,'hold');
    CamPlot(cam,opt.pstar,'color','blue','style','o','size',5,'hold');
 
    // create trajectory for each point
    u1 = squeeze(pmat(1,1,:));
    v1 = squeeze(pmat(2,1,:));
    pt.u1 = [p0(1,1); u1];
    pt.v1 = [p0(2,1); v1];

    u2 = squeeze(pmat(1,2,:));
    v2 = squeeze(pmat(2,2,:));
    pt.u2 = [p0(1,2); u2];
    pt.v2 = [p0(2,2); v2];    
    
    u3 = squeeze(pmat(1,3,:));
    v3 = squeeze(pmat(2,3,:));
    pt.u3 = [p0(1,3); u3];
    pt.v3 = [p0(2,3); v3];
    
    u4 = squeeze(pmat(1,4,:));
    v4 = squeeze(pmat(2,4,:));
    pt.u4 = [p0(1,4); u4];
    pt.v4 = [p0(2,4); v4];
            
    figure(1);
    plot(pt.u1,pt.v1,'r-',pt.u2,pt.v2,'r-',pt.u3,pt.v3,'r-',pt.u4,pt.v4,'r-');
    
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
    
    if opt.depthest  // plot depth estimation
        figure(6);
        plot(tk, zvec(:,2:size(zvec,2)));
        xlabel('time');
        ylabel('Z');
        title('Depth estimation');
    end    
                
   
endfunction
