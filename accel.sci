//accel.sci Manipulator forward dynamics
//
// QDD = accel(robot, Q, QD, TORQUE) is a vector (Nx1) of joint accelerations that result 
// from applying the actuator force/torque to the manipulator robot in state Q and QD.
// If Q, QD, TORQUE are matrices (KxN) then QDD is a matrix (KxN) where each row 
// is the acceleration corresponding to the equivalent rows of Q, QD, TORQUE.
//
// QDD = accel(robot, X) as above but X=[Q,QD,TORQUE].
//
// Note::
// - Uses the method 1 of Walker and Orin to compute the forward dynamics.
// - This form is useful for simulation of manipulator dynamics, in
//   conjunction with a numerical integration function.
//
// References::
// - Efficient dynamic computer simulation of robotic mechanisms,
//   M. W. Walker and D. E. Orin,
//   ASME Journa of Dynamic Systems, Measurement and Control, vol. 104, no. 3, pp. 205-211, 1982.
//
// See also rne( ), SerialLink( ), ode45.
//
// This file is adapted from The Robotics Toolbox for Matlab (RTB).
// Copyright (C) 1993-2011, by Peter I. Corke
// 


function qdd = accel(robot, Q, qd, torque)

	n = robot.nj;
    nargin = argn(2);
	if nargin == 2
        // accel(X)
        Q = Q(:)';   // make it a row vector
	    q = Q(1:n);
		qd = Q(n+1:2*n);
		torque = Q(2*n+1:3*n);
    elseif nargin == 4
        // accel(Q, qd, torque)
        
        if numrows(Q) > 1
            if numrows(Q) ~= numrows(qd)
                error('for trajectory q and qd must have same number of rows');
            end
            if numrows(Q) ~= numrows(torque)
                error('for trajectory q and torque must have same number of rows');
            end
            qdd = [];
            for i=1:numrows(Q)
                qdd = cat(1, qdd, accel(robot, Q(i,:), qd(i,:), torque(i,:))');
            end
            return
        else
            q = Q';
            if length(q) == n
                q = q(:)';
                qd = qd(:)';
            end
            if numcols(Q) ~= n
                msg = sprintf('q must have %d columns', n);
                error(msg);
            end
            if numcols(qd) ~= n
                msg = sprintf('qd must have %d columns', n);
                error(msg);
            end
            if numcols(torque) ~= n
                msg = sprintf('torque must have %d columns', n);
                error(msg);
            end
        end
    else
        error('Insufficient arguments');
    end


	// compute current manipulator inertia
	//   torques resulting from unit acceleration of each joint with
	//   no gravity.
	M = rne(robot, ones(n,1)*q, zeros(n,n), eye(n,n), [0;0;0]);

	// compute gravity and coriolis torque
	//    torques resulting from zero acceleration at given velocity &
	//    with gravity acting.
	tau = rne(robot, q, qd, zeros(1,n));	

	qdd = M \ (torque - tau)';
endfunction
