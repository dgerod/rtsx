//ISHOMOG Test if argument is a homogeneous transformation
//
// ISHOMOG(T) is true (1) if the argument T is of dimension 4x4 or 4x4xN, else 
// false (0).
//
// ISHOMOG(T, 'valid') as above, but also checks the validity of the rotation
// matrix.
//
// See also ISROT, ISVEC.


// Copyright (C) 1993-2011, by Peter I. Corke
//
// This file is part of The Robotics Toolbox for Matlab (RTB).
// 
//
function h = ishomog(tr, rtest)
    d = size(tr);
    if ndims(tr) >= 2
        h =  and(d(1:2) == [4 4]);

        if h & argn(2) > 1
            h = abs(det(tr(1:3,1:3)) - 1) < 1d-15; // %eps;
        end

    else
        h = 0;
    end
endfunction