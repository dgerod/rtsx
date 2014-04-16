//JACOB0.sci Jacobian in world coordinates
// www.controlsystemslab.com  August 2012
//
// J0 = jacob0(robot,Q, OPTIONS) is the Jacobian matrix (6xN) for the robot in 
// pose Q (1xN).  The manipulator Jacobian matrix maps joint velocity to 
// end-effector spatial velocity V = J0*QD expressed in the world-coordinate 
// frame.
//
// Options::
// 'rpy'     Compute analytical Jacobian with rotation rate in terms of 
//           roll-pitch-yaw angles
// 'eul'     Compute analytical Jacobian with rotation rates in terms of 
//           Euler angles
// 'trans'   Return translational submatrix of Jacobian
// 'rot'     Return rotational submatrix of Jacobian 
//
// Note::
// - The Jacobian is computed in the world frame and transformed to the 
//   end-effector frame.
// - The default Jacobian returned is often referred to as the geometric 
//   Jacobian, as opposed to the analytical Jacobian.
//
// See also SerialLink.jacobn, jsingu, deltatr, tr2delta, jsingu.


//
// This function is adapted from The Robotics Toolbox for Matlab (RTB).
// Copyright (C) 1993-2011, by Peter I. Corke
// http://www.petercorke.com

function J0 = jacob0(robot, q, varargin)

    opt.rpy = false;
    opt.eul = false;
    opt.trans = false;
    opt.rot = false;
    
    //opt = tb_optparse(opt, varargin);
    varnum=length(varargin);  // number of variable arguments    
    for iv =1:varnum
 
        if type(varargin(iv))==10 then  // string
            varargin(iv)=convstr(varargin(iv),'l');   // convert to lower case
        end
        if varargin(iv)=='rpy' then
            opt.rpy = true;       
        elseif varargin(iv)== 'eul' then
            opt.eul = true;
        elseif varargin(iv)== 'trans' then
            opt.trans = true;
        elseif varargin(iv)== 'rot' then
            opt.rot = true;

        end    // if varargin(iv)=='rpy'
    end    // for iv = 1:varnum
	//
	//   dX_tn = Jn dq
	//
	Jn = jacobn(robot, q);	// Jacobian from joint to wrist space

	//
	//  convert to Jacobian in base coordinates
	//
	Tn = fkine(robot, q);	// end-effector transformation
	R = t2r(Tn);
	J0 = [R zeros(3,3); zeros(3,3) R] * Jn;

    if opt.rpy
        rpy = tr2rpy( fkine(robot, q) );
        B = rpy2jac(rpy);
        if rcond(B) < %eps
            error('Representational singularity');
        end
        J0 = blkdiag2( eye(3,3), inv(B) ) * J0;
    elseif opt.eul
        eul = tr2eul( fkine(robot, q) );
        B = eul2jac(eul);
        if rcond(B) < %eps
            error('Representational singularity');
        end
        J0 = blkdiag2( eye(3,3), inv(B) ) * J0;
    end
    
    if opt.trans
        J0 = J0(1:3,:);
    elseif opt.rot
        J0 = J0(4:6,:);
    end
    J0 = clean(J0);
endfunction
