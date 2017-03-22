//inertia.sci Manipulator inertia matrix
//
// I = inertia(robot, Q) is the symmetric joint inertia matrix (NxN) which relates 
// joint torque to joint acceleration for the robot at joint configuration Q.
//
// If Q is a matrix (KxN), each row is interpretted as a joint state 
// vector, and the result is a 3d-matrix (NxNxK) where each plane corresponds
// to the inertia for the corresponding row of Q.
//
// Notes::
// - The diagonal elements I(J,J) are the inertia seen by joint actuator J.
// - The off-diagonal elements I(J,K) are coupling inertias that relate 
//   acceleration on joint J to force/torque on joint K.
// - The diagonal terms include the motor inertia reflected through the gear
//   ratio.
//



// Copyright (C) 1993-2011, by Peter I. Corke
//
// This file is adapted from The Robotics Toolbox for Matlab (RTB).
// Copyright (C) 1993-2011, by Peter I. Corke


function M = inertia(robot, q)
    if numcols(q) ~= robot.nj
        error('q must have //d columns', robot.nj);
    end

    if numrows(q) > 1
        M = [];
        for i=1:numrows(q)
            M = cat(3, M, inertia(robot, q(i,:)));
        end
        return
    end

	n = robot.nj;

	if length(q) == n
		q = q(:)';
	end

	M = zeros(n,n,0);
	for Q = q'
		m = rne(robot, ones(n,1)*Q', zeros(n,n), eye(n,n), [0;0;0]);
		M = cat(3, M, m);
	end
endfunction


//cinertia() Cartesian inertia matrix
//
// M = cinertia(robot, Q) is the NxN Cartesian (operational space) inertia matrix which relates 
// Cartesian force/torque to Cartesian acceleration at the joint configuration Q, and N 
// is the number of robot joints.
//
// See also SerialLink.inertia, SerialLink.rne.

// MOD HISTORY
// 	4/99 add object support
// $Log: not supported by cvs2svn $
// $Revision: 1.2 $


function Mx = cinertia(robot, q)
	J = jacob0(robot, q);
	Ji = inv(J);                //#ok<*MINV>
	M = inertia(robot, q);
	Mx = Ji' * M * Ji;
endfunction
