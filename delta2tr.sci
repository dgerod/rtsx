//DELTA2TR Convert differential motion  to a homogeneous transform
//
// T = DELTA2TR(D) is a homogeneous transform representing differential 
// translation and rotation. The vector D=(dx, dy, dz, dRx, dRy, dRz)
// represents an infinitessimal motion, and is an approximation to the spatial 
// velocity multiplied by time.
//
// See also TR2DELTA.

// This file is adapted from The Robotics Toolbox for Matlab (RTB).
// Copyright (C) 1993-2011, by Peter I. Corke
 
function T = delta2tr(d)
    //d = d(:);
    T = eye(4,4) + [skew(d(4:6)) d(1:3); 0 0 0 0];
endfunction