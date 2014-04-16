//ISVEC Test if argument is a vector
//
// ISVEC(V) is true (1) if the argument V is a 3-vector, else false (0).
//
// ISVEC(V, L) is true (1) if the argument V is a vector of length L,
// either a row- or column-vector.  Otherwise false (0).
//
// Notes::
// - differs from MATLAB builtin function ISVECTOR, the latter returns true
//   for the case of a scalar, ISVEC does not.
//
// See also ISHOMOG, ISROT.

// Copyright (C) 1993-2011, by Peter I. Corke
//
// This file is part of The Robotics Toolbox for Matlab (RTB).


function h = isvec(v, l)
    if argn(2) == 1
            l = 3;
    end
    d = size(v);
    h = length(d) == 2 & min(d) == 1 & length(v) == l;
endfunction
