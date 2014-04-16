//TR2DELTA Convert homogeneous transform to differential motion
//
// D = TR2DELTA(T0, T1) is the differential motion (6x1) corresponding to 
// infinitessimal motion from pose T0 to T1 which are homogeneous 
// transformations. D=(dx, dy, dz, dRx, dRy, dRz) and is an approximation
// to the average spatial velocity multiplied by time.
//
// D = TR2DELTA(T) is the differential motion corresponding to the
// infinitessimal relative pose T expressed as a homogeneous 
// transformation.
//
// Notes::
// - D is only an approximation to the motion T, and assumes
//   that T0 ~ T1 or T ~ eye(4,4).
//
// See also DELTA2TR, SKEW.



// This function is adapted from The Robotics Toolbox for Matlab (RTB).
// Copyright (C) 1993-2011, by Peter I. Corke
//

function delta = tr2delta(T0, T1)
    nargin = argn(2);
    if nargin == 1
        T1 = T0;
        T0 = eye(4,4);
    end
    R0 = t2r(T0); R1 = t2r(T1);
    // in world frame
    //pause;
    delta = [ (T1(1:3,4)-T0(1:3,4)); vex( R1*R0' - eye(3,3)) ];
    // in T0 frame
    //delta = [ R0'*(T1(1:3,4)-T0(1:3,4)); R0'*vex( R1*R0' - eye(3,3)) ];

// TODO HACK understand/fix this and update Chapter 2
//    delta = [	T1(1:3,4)-T0(1:3,4);
//        0.5*(	cross(T0(1:3,1), T1(1:3,1)) + ...
//            cross(T0(1:3,2), T1(1:3,2)) + ...
//            cross(T0(1:3,3), T1(1:3,3)) ...
//        )];
endfunction

