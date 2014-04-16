// vex.sci  convert skew-symmetric matrix to vector
// www.controlsystemslab.com  August 2012
// V = VEX(S) is the vector (3x1) which has the skew-symmetric matrix S (3x3)
//
//           | 0   -vz  vy|
//           | vz   0  -vx|
//           |-vy   vx  0 |
//
// Notes::
// - This is the inverse of the function SKEW().
// - No checking is done to ensure that the matrix is actually skew-symmetric.
// - The function takes the mean of the two elements that correspond to each unique
//   element of the matrix, ie. vx = 0.5*(S(3,2)-S(2,3))
//
// See also SKEW.
// This function is adapted from The Robotics Toolbox for Matlab (RTB).
// Copyright (C) 1993-2011, by Peter I. Corke

function v = vex(S)
    
    if isrot(S) | ishomog(S)
        v = 0.5*[S(3,2)-S(2,3); S(1,3)-S(3,1); S(2,1)-S(1,2)];
    else
        error('argument must be a 3x3 matrix');
    end    
    
//    V=zeros(3,1);

//    V(1)= S(3,2);
//    V(2) = S(1,3);
//    V(3) = S(2,1);

    
endfunction