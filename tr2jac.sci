//TR2JAC Jacobian for differential motion
//
// J = TR2JAC(T) is a Jacobian matrix (6x6) that maps spatial velocity or
// differential motion from the world frame to the frame represented by 
// the homogeneous transform T.
//
// See also WTRANS, TR2DELTA, DELTA2TR.

// This file is adapted from The Robotics Toolbox for Matlab (RTB).
// Copyright (C) 1993-2011, by Peter I. Corke
 
function J = tr2jac(T)
		
    R = t2r(T);
    J = [
                 R'  (skew(t2d(T))*R)'
        zeros(3,3)                     R'
        ];
    J = clean(J);
endfunction