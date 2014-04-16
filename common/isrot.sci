//ISROT Test if argument is a rotation matrix
//
// ISROT(R) is true (1) if the argument is of dimension 3x3 or 3x3xN, else false (0).
//
// ISROT(R, 'valid') as above, but also checks the validity of the rotation
// matrix.
//
// See also ISHOMOG, ISVEC.
// modified from The Robotics Toolbox for Matlab (RTB).

// Copyright (C) 1993-2011, by Peter I. Corke

function h = isrot(r, dtest)

    d = size(r);
    if ndims(r) >= 2
        h =  and(d(1:2) == [3 3]);

        if h & (argn(2) > 1)
            h = abs(det(r) - 1) < %eps;
        end

    else
        h = 0;
    end
endfunction