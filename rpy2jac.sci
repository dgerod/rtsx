//RPY2JAC.sci Jacobian from RPY angle rates to angular velocity
// www.controlsystemslab.com  August 2012
//
// J = RPY2JAC(rpy) is a Jacobian matrix (3x3) that maps roll-pitch-yaw angle 
// rates to angular velocity at the operating point RPY=[R,P,Y].
//
// J = RPY2JAC(R, P, Y) as above but the roll-pitch-yaw angles are passed
// as separate arguments.
//
// Notes::
// - Used in the creation of an analytical Jacobian.
//
// See also EUL2JAC, SerialLink.JACOBN.


// This function is adapted from The Robotics Toolbox for Matlab (RTB).
// Copyright (C) 1993-2011, by Peter I. Corke
// http://www.petercorke.com

function J = rpy2jac(r, p, y)

    if length(r) == 3
        // rpy2jac([r,p,y])
        p = r(2);
        y = r(3);
        r = r(1);
    end
	J = [	
        1  0       sin(p)
        0  cos(r)  -cos(p)*sin(r)
        0  sin(r)  cos(p)*cos(r)
        ];
    J = clean(J);
		
endfunction