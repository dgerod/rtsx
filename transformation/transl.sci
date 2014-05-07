// TRANSL.SCI  basic homogeneous transformation for translation
// www.controlsystemslab.com  July 2012
// translate a point or coordinate frame to new location p
// Example:
// p = [2 3 -1]';
// T = transl(p);

//function T=transl(p)
//    if size(p)== [1 3] then
//        p = p';
//    end
//    T=[eye(3,3) p; 0 0 0 1]
//endfunction

// April 2013: use the code adapted from The Robotics Toolbox for Matlab (RTB).
// Copyright (C) 1993-2011, by Peter I. Corke
////TRANSL Create translational transform
////
//// T = TRANSL(X, Y, Z) is a homogeneous transform representing a 
//// pure translation.
////
//// T = TRANSL(P) is a homogeneous transform representing a translation or 
//// point P=[X,Y,Z]. If P (Mx3) it represents a sequence and T (4x4xM)
//// is a sequence of homogenous transforms such that T(:,:,i) corresponds to
//// the i'th row of P.
////
//// P = TRANSL(T) is the translational part of a homogenous transform as a 
//// 3-element column vector.  If T (4x4xM) is a homgoeneous transform sequence 
//// the rows of P (Mx3) are the translational component of the corresponding 
//// transform in the sequence.
////
//// Notes::
//// - Somewhat unusually this function performs a function and its inverse.  An
////   historical anomaly.

function T = transl(x, y, z)
    nargin = argn(2);
    if nargin == 1
        if ishomog(x)
            if ndims(x) == 3
                // transl(T)  -> P, trajectory case
                T = squeeze(x(1:3,4,:))';
            else
                // transl(T)  -> P
                T = x(1:3,4);
            end
        elseif and(size(x) == [3 3])
            T = x(1:2,3);
        elseif length(x) == 2
            // transl(P) -> T
            t = x(:);
            T =    [eye(2,2)          t(:);
                0   0   1];
        elseif length(x) == 3
            // transl(P) -> T
            t = x(:);
            T =    [eye(3,3)          t(:);
                0   0   0   1];
        else
            // transl(P) -> T, trajectory case
            n = numrows(x);
            T = repmat(eye(4,4), [1 1 n]);
            T(1:3,4,:) = x';
        end    
    elseif nargin == 2
        // transl(x,y) -> T
        t = [x; y];
        T =    rp2t( eye(2,2), t);        
    elseif nargin == 3
        // transl(x,y,z) -> T
        t = [x; y; z];
        T =    rp2t( eye(3,3), t);
    end
endfunction
