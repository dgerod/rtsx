//JTRAJ Compute a joint space trajectory between two points
//
// [Q,QD,QDD] = JTRAJ(Q0, QF, M) is a joint space trajectory Q (MxN) where the joint
// coordinates vary from Q0 (1xN) to QF (1xN).  A quintic (5th order) polynomial is used 
// with default zero boundary conditions for velocity and acceleration.  
// Time is assumed to vary from 0 to 1 in M steps.  Joint velocity and 
// acceleration can be optionally returned as QD (MxN) and QDD (MxN) respectively.
// The trajectory Q, QD and QDD are MxN matrices, with one row per time step,
// and one column per joint.
//
// [Q,QD,QDD] = JTRAJ(Q0, QF, M, QD0, QDF) as above but also specifies initial 
// and final joint velocity for the trajectory.
//
// [Q,QD,QDD] = JTRAJ(Q0, QF, T) as above but the trajectory length is defined
// by the length of the time vector T (Mx1).
//
// [Q,QD,QDD] = JTRAJ(Q0, QF, T, QD0, QDF) as above but specifies initial and 
// final joint velocity for the trajectory and a time vector.
//
// See also CTRAJ, SerialLink.jtraj.
//
// This file is adapted from The Robotics Toolbox for Matlab (RTB).
// Copyright (C) 1993-2011, by Peter I. Corke

function [qt,qdt,qddt] = jtraj(q0, q1, tv, qd0, qd1)
    nargout = argn(1);
    nargin = argn(2);
    if length(tv) > 1
        tscal = max(tv);
        t = tv(:)/tscal;
    else
        tscal = 1;
        t = (0:(tv-1))'/(tv-1); // normalized time from 0 -> 1
    end

    q0 = q0(:);
    q1 = q1(:);

    if nargin == 3
        qd0 = _zeros(size(q0));  //zeros(size(q0,1),size(q0,2));
        qd1 = qd0;
    elseif nargin == 5
        qd0 = qd0(:);
        qd1 = qd1(:);
    else
        error('incorrect number of arguments')
    end
   //pause;
    // compute the polynomial coefficients
    A = 6*(q1 - q0) - 3*(qd1+qd0)*tscal;
    B = -15*(q1 - q0) + (8*qd0 + 7*qd1)*tscal;
    C = 10*(q1 - q0) - (6*qd0 + 4*qd1)*tscal;
    E = qd0*tscal; // as the t vector has been normalized
    F = q0;

    tt = [t.^5 t.^4 t.^3 t.^2 t _ones(size(t))];
    c = [A B C _zeros(size(A)) E F]';
    //pause;
    qt = tt*c;

    // compute optional velocity
    if nargout >= 2
        c = [ _zeros(size(A)) 5*A 4*B 3*C  _zeros(size(A)) E ]';
        qdt = tt*c/tscal;
    end

    // compute optional acceleration
    if nargout == 3
        c = [ _zeros(size(A))  _zeros(size(A)) 20*A 12*B 6*C  _zeros(size(A))]';
        qddt = tt*c/tscal^2;
    end
endfunction


