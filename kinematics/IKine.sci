// ikine.sci   Inverse mainipulator kinematics
// www.controlsystemslab.com  August 2012
//
// q = ikine(robot,T) is the joint coordinates corresponding to the robot 
// end-effector pose T which is a homogenenous transform.
//
// q = ikine(robot, T, options) specifies the initial estimate of the joint 
// coordinates, masks
// q = ikine(robot, T, options) specifies the initial estimate of the joint 
// coordinates and a mask matrix.  For the case where the manipulator 
// has fewer than 6 DOF the solution space has more dimensions than can
// be spanned by the manipulator joint coordinates.  In this case
// the mask matrix M specifies the Cartesian DOF (in the wrist coordinate 
// frame) that will be ignored in reaching a solution.  The mask matrix 
// has six elements that correspond to translation in X, Y and Z, and rotation 
// about X, Y and Z respectively.  The value should be 0 (for ignore) or 1.
// The number of non-zero elements should equal the number of manipulator DOF.
//
// For example when using a 5 DOF manipulator rotation about the wrist z-axis
// might be unimportant in which case  M = [1 1 1 1 1 0].
//
// In all cases if T is 4x4xM it is taken as a homogeneous transform sequence 
// and ikine() returns the joint coordinates corresponding to each of the 
// transforms in the sequence.  Q is MxN where N is the number of robot joints.
// The initial estimate of Q for each time step is taken as the solution 
// from the previous time step.
//
// Options::
// 'q0', q0 --- initial estimate of joint coordinate
// 'm', M  --- mask matrix
// 'pinv'         use pseudo-inverse instead of Jacobian transpose
// 'ilimit',L     set the maximum iteration count (default 1000)
// 'tol',T        set the tolerance on error norm (default 1e-6)
// 'alpha',A      set step size gain (default 1)
// 'novarstep'    disable variable step size
// 'verbose'      show number of iterations for each point
// 'verbose=2'    show state at each iteration
// 'plot'         plot iteration state versus time
//
// Notes::
// - Solution is computed iteratively.
// - Solution is sensitive to choice of initial gain.  The variable
//   step size logic (enabled by default) does its best to find a balance
//   between speed of convergence and divergence.
// - Some experimentation might be required to find the right values of 
//   tol, ilimit and alpha.
// - The pinv option sometimes leads to much faster convergence.
// - The tolerance is computed on the norm of the error between current
//   and desired tool pose.  This norm is computed from distances
//   and angles without any kind of weighting.
// - The inverse kinematic solution is generally not unique, and 
//   depends on the initial guess Q0 (defaults to 0).
// - Such a solution is completely general, though much less efficient 
//   than specific inverse kinematic solutions derived symbolically, like
//   ikine6s or ikine3.
// - This approach allows a solution to obtained at a singularity, but 
//   the joint angles within the null space are arbitrarily assigned.
// - Joint offsets, if defined, are added to the inverse kinematics to 
//   generate Q.
//
// See also fkine, tr2delta, SerialLink.jacob0, ikine6s. 
//
// This function is adapted from The Robotics Toolbox for Matlab (RTB).
// Copyright (C) 1993-2011, by Peter I. Corke 
// http://www.petercorke.com

function [qt, histout]=IKine(robot, tr, varargin)
    [qt,histout]=_ikine(robot, tr, varargin);
endfunction

function [qt, histout]=ikine(robot, tr, varargin)
    [qt, histout]=_ikine(robot, tr, varargin);
endfunction

function [qt,histout] = _ikine(robot, tr, varargin)
    varargin = varargin($);
    false = 0;
    true = 1;
    gotmask = 0;    // a flag to check whether a mask is passed
    //  set default parameters for solution
    opt.ilimit = 1000;
    opt.tol = 1e-6;
    opt.alpha = 1;
    opt.plot = false;
    opt.pinv = false;
    opt.varstep = true;
    opt.verbose = 0;
    histout = [];
    varnum=length(varargin);  // number of variable arguments
    
    n = robot.nj;
    if ndims(tr)== 3
        nseqs = size(tr,3);   //  number of sequence
        q = zeros(nseqs,n);   // initial joint variable value
    else
        nseqs = 1;
        q = zeros(1,n);    // initial joint variable value
    end
    m = ones(1,6);    // mask

    for iv =1:varnum
 
        if type(varargin(iv))==10 then  // string
            varargin(iv)=convstr(varargin(iv),'l');   // convert to lower case
        end
        if varargin(iv)=='pinv' then
            opt.pinv = true;       
        elseif varargin(iv)== 'novarstep' then
            opt.varstep = false;
        elseif varargin(iv)== 'plot' then
            if ndims(tr)>2    // plot option not applicable for 3-D tr
                error("Cannot use plot option with sequence operations");
            else
                opt.plot = true;
            end
        elseif varargin(iv)== 'verbose' then
            opt.verbose = 1;
        elseif varargin(iv)== 'verbose=2' then
            opt.verbose = 2;    
        elseif varargin(iv)=='q0'& (iv<varnum) then  // initial j.v values
            if type(varargin(iv+1))== 1 & isequal(size(varargin(iv+1)),[nseqs n])    
                q = varargin(iv+1); 
            else
                msg = sprintf("q0 must be a  %d x %d vector/matrix",nseqs, n);
                error(msg);
            end  
        elseif varargin(iv)=='m'& (iv<varnum) then  // mask
            if type(varargin(iv+1))== 1 & isequal(size(varargin(iv+1)),[1 6])    
                m = varargin(iv+1);
                gotmask = 1;        // a mask is passed 
                if length(find(m)) ~= n
                    error('Mask matrix must have same number of 1s as robot DOF');
                end
            else
                msg = sprintf("mask must be a 1 x 6 vector");
                error(msg);
            end                      
        elseif varargin(iv)=='ilimit'& (iv<varnum) then  // max iteration count
            if type(varargin(iv+1))== 1,    // must be a number
                opt.ilimit = varargin(iv+1); 
            else
                error("ilimit: maximum iteration count must be a number");
            end
        elseif varargin(iv)=='tol' & (iv<varnum) then  // tolerance on error norm
            if type(varargin(iv+1))== 1,    // must be a number
                opt.tol = varargin(iv+1); 
            else
                error("tol: tolerance on error norm must be a number");
            end
        elseif varargin(iv)=='alpha' & (iv<varnum) then  // step size gain
            if type(varargin(iv+1))== 1,    // must be a number
                opt.alpha = varargin(iv+1); 
            else
                error("alpha: step size gain must be a number");
            end            

        end    // if varargin(iv) == 'pinv'
    end

    //[opt,args] = tb_optparse(opt, varargin);

    if n<6 & ~gotmask,
 
        error('For a manipulator with fewer than 6DOF a mask matrix argument must be specified');
    end

    // make this a logical array so we can index with it
    //m = logical(m);
    m = (m & [1 1 1 1 1 1]);

    npoints = nseqs; //size(tr,3);    // number of points
    qt = zeros(npoints, n);  // preallocate space for results
    tcount = 0;              // total iteration count

    
    history = [];
    failed = false;
    for i=1:npoints
        if ndims(tr)==3
            T = tr(:,:,i);
        else T = tr;
        end
        bignum = 1e20;
        nm = bignum;
        // initialize state for the ikine loop
        eprev = bignum;
        saved.e = bignum*ones(1,n);
        saved.q = [];
        count = 0;
        printf('Computing inverse kinematics for robot %s\n',robot.name);
        printf('\n');
        while true
            if opt.verbose == 0 
                printf('\r%d',count);    // see progress
            end
                
            // update the count and test against iteration limit
            count = count + 1;
            if count > opt.ilimit
                msg = sprintf("ikine: iteration limit %d exceeded (row %d), final err %f', ...
                    opt.ilimit, i, nm);
                warning(msg);
                failed = true;
                //q = %nan*ones(1,n);
                break;
            end
            //pause;
            // compute the error
            e = tr2delta(fkine(robot, q), T);

            
            //pause;
            // optionally adjust the step size
            if opt.varstep
                // test against last best error, only consider the DOF of
                // interest

                if norm(e(m)) < norm(saved.e(m))
                    // error reduced,
                    // let's save current state of solution and rack up the step size
                    saved.q = q;
                    saved.e = e;
                    opt.alpha = opt.alpha * (2.0^(1.0/8));
                    if opt.verbose > 1
                        printf('raise alpha to %f\n', opt.alpha);
                    end
                else
                    // rats!  error got worse,
                    // restore to last good solution and reduce step size
                    q = saved.q;
                    e = saved.e;
                    opt.alpha = opt.alpha * 0.5;
                    if opt.verbose > 1
                        printf('drop alpha to %f\n', opt.alpha);
                    end
                end  // if norm(e(m)) < norm(saved.e(m))
            end  // if opt.varstep

            // compute the Jacobian
            //pause;
            J = jacob0(robot, q);

            //disp(J);
            // compute change in joint angles to reduce the error, 
            // based on the square sub-Jacobian
            if opt.pinv

                dq = opt.alpha * pinv( J(m,:) ) * e(m);
            else

                dq = J(m,:)' * e(m);
                dq = opt.alpha * dq;
            end

            // diagnostic stuff
            if opt.verbose > 1
                printf('%d:%d: |e| = %f\n', i, count, nm);
                printf('       e  = '); disp(e');
                printf('       dq = '); disp(dq');
            end
            if opt.plot
                history.q(count,:) = q;
                history.dq(count,:) = dq';
                history.e(count,:) = e';
                history.ne(count) = nm;
                history.alpha(count) = opt.alpha;
                
            end

            // update the estimated solution
            q = q + dq';
            nm = norm(e(m));

            if norm(e) > 1.5*norm(eprev)
                warning('RTB:ikine:diverged -- solution diverging, try reducing alpha');
            end
            eprev = e;

            if nm <= opt.tol
                break;
            end

        end  // end ikine solution for tr(:,:,i)
        qt(i,:) = q;
        tcount = tcount + count;
        if opt.verbose
            printf('/%d iterations\n', count);
        end
    end
    
    if opt.verbose & npoints > 1
        printf('TOTAL %d iterations\n', tcount);
    end

    // plot evolution of variables
    if opt.plot
        figure(1);
        plot(history.q);
        xlabel('iteration');
        ylabel('q');
        
        
        xgrid

        figure(2);
        plot(history.dq);
        xlabel('iteration');
        ylabel('dq');
       
        xgrid

        figure(3);
        plot(history.e);
        xlabel('iteration');
        ylabel('e');
        
        xgrid

        figure(4);
        plot2d("nl", history.ne);
        xlabel('iteration');
        ylabel('|e|');
        xgrid

        figure(5);
        plot(history.alpha);
        xlabel('iteration');
        ylabel('alpha');
        xgrid;
        nargout=argn(1);
        if nargout > 1
            histout = history;
        end
    end
    printf('\n');
endfunction
