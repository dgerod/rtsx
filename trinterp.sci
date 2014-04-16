//TRINTERP Interpolate homogeneous transformations
//
// T = TRINTERP(T0, T1, S) is a homogeneous transform interpolation 
// between T0 when S=0 to T1 when S=1.  Rotation is interpolated using 
// quaternion spherical linear interpolation.  If S (Nx1) then T (4x4xN)
// is a sequence of homogeneous transforms corresponding to the interpolation 
// values in S.
//
// T = TRINTERP(T, S) is a transform that varies from the identity matrix when
// S=0 to T when R=1.  If S (Nx1) then T (4x4xN) is a sequence of homogeneous 
// transforms corresponding to the interpolation values in S.
//
// See also CTRAJ, QUATERNION.
//
// This file is adapted from The Robotics Toolbox for Matlab (RTB).
// Copyright (C) 1993-2011, by Peter I. Corke

function T = trinterp(A, B, C)
   nargin = argn(2);
    if nargin == 3
        //	TR = TRINTERP(T0, T1, r)
        T0 = A; T1 = B; r = C;

        if length(r) > 1
            T = [];
            for rr=r(:)'
                TT = trinterp(T0, T1, rr);
                T = cat(3, T, TT);
            end
            return;
        end

        q0 = Quaternion(T0);
        q1 = Quaternion(T1);

        p0 = t2p(T0);
        p1 = t2p(T1);

        qr = qinterp(q0,q1, r);
        pr = p0*(1-r) + r*p1;
    elseif nargin == 2
    //	TR = TRINTERP(T, r)
        T0 = A; r = B;

        if length(r) > 1
            T = [];
            for rr=r(:)'
                TT = trinterp(T0, rr);
                T = cat(3, T, TT);
            end
            return;
        end

        q0 = Quaternion(T0);
        p0 = t2p(T0);

        qr = qscale(q0,r);
        pr = r*p0;
    else
        error('must be 2 or 3 arguments');
    end
    //T = rt2tr(qr.R, pr);
     T = clean(rp2t(q2r(qr), pr));   
endfunction
