//gravload.sci 
//
// TAUG = gravload(robot, Q) is the joint gravity loading for the robot in the
// joint configuration Q.  Gravitational acceleration is a property of the
// robot object.
//
// If Q is a row vector, the result is a row vector of joint torques.  If 
// Q is a matrix, each row is interpreted as a joint configuration vector, 
// and the result is a matrix each row being the corresponding joint torques.
//
// TAUG = gravload(robot, Q, GRAV) is as above but the gravitational 
// acceleration vector GRAV is given explicitly.
//
// See also rne(), itorque(), coriolis().
//
// This file is adapted from The Robotics Toolbox for Matlab (RTB).
// Copyright (C) 1993-2011, by Peter I. Corke


function tg = gravload(robot, q, grav)
    nargin = argn(2);
	if numcols(q) ~= robot.nj
		error('number of columns in q does not match number of robot links')
	end
	if nargin == 2
		tg = rne(robot, q, _zeros(size(q)), _zeros(size(q)));
	elseif nargin == 3
		tg = rne(robot, q, _zeros(size(q)), _zeros(size(q)), grav);
	end
endfunction	
