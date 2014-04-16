//SKEW Create skew-symmetric matrix
//
// S = SKEW(V) is a skew-symmetric matrix formed from V (3x1).
//
//           | 0   -vz  vy|
//           | vz   0  -vx|
//           |-vy   vx  0 |
//
// See also VEX.
//
// This file is adapted from The Robotics Toolbox for Matlab (RTB).
// Copyright (C) 1993-2011, by Peter I. Corke

function S = skew(v)
    if isvec(v,3)
        // SO(3) case
        S = [  0   -v(3)  v(2)
              v(3)  0    -v(1)
             -v(2) v(1)   0];
    elseif isvec(v,1)
        // SO(2) case
        S = [0 -v; v 0];
    else
        error('argument must be a 1- or 3-vector');
    end
