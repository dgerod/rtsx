//CTRAJ Cartesian trajectory between two points
//
// TC = CTRAJ(T0, T1, N) is a Cartesian trajectory (4x4xN) from pose T0 to T1
// with N points that follow a trapezoidal velocity profile along the path.
// The Cartesian trajectory is a homogeneous transform sequence and the last 
// subscript being the point index, that is, T(:,:,i) is the i'th point along
// the path.
//
// TC = CTRAJ(T0, T1, S) as above but the elements of S (Nx1) specify the 
// fractional distance  along the path, and these values are in the range [0 1].
// The i'th point corresponds to a distance S(i) along the path.
//
// See also LSPB, MSTRAJ, TRINTERP, qinterp, TRANSL.

// This file is adapted from of The Robotics Toolbox for Matlab (RTB).
// Copyright (C) 1993-2011, by Peter I. Corke
// 

function traj = ctraj(T0, T1, t)

    if ~ishomog(T0) | ~ishomog(T1)
        error('arguments must be homogeneous transformations');
    end
    
    // distance along path is a smooth function of time
    if isscalar(t)
        s = lspb(0, 1, t);
    else
        s = t(:);
    end

    traj = [];

    for S=s'
        traj = cat(3, traj, trinterp(T0, T1, S));
    end
endfunction
    

