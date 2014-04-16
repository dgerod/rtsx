//EUL2JAC.sci Euler angle rate Jacobian
// www.controlsystemslab.com  August 2012
//
// J = EUL2JAC(EUL) is a Jacobian matrix (3x3) that maps Euler angle rates to 
// angular velocity at the operating point EUL=[PHI, THETA, PSI]. 
//
// J = EUL2JAC(PHI, THETA, PSI) as above but the Euler angles are passed
// as separate arguments.
//
// Notes::
// - Used in the creation of an analytical Jacobian.
//
// See also RPY2JAC, JACOBN.


// This function is adapted from The Robotics Toolbox for Matlab (RTB).
// Copyright (C) 1993-2011, by Peter I. Corke
// http://www.petercorke.com


function J = eul2jac(phi, theta, psi)

    if length(phi) == 3
        // eul2jac([phi theta psi])
        theta = phi(2);
        psi = phi(3);
        phi = phi(1);
    end
    J = [   
        cos(psi)*sin(theta)   -sin(psi)    0
        sin(psi)*sin(theta)  cos(psi)      0
        cos(theta)           0             1
        ];
    J = clean(J);
        
endfunction