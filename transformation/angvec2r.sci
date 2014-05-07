//ANGVEC2R Convert angle and vector orientation to a rotation matrix
//
// R = ANGVEC2R(THETA, V, 'options') is an rthonormal rotation matrix, R, 
// equivalent to a rotation of THETA about the vector V.
//
// See also eul2r, rpy2r.

// This file is adapted from The RObotics Toolbox for MATLAB (RTS)
// Copyright (C) 1993-2011, by Peter I. Corke

function R = angvec2r(theta, k, varargin)
    opt.deg = 0;

    // process varargin 
    numopts=length(varargin); // find number of options
    if numopts>0 then
        for i=1:numopts
            if varargin(i)== 'deg' then opt.deg = 1;
            end
        end
    end
    // optionally convert from degrees
    if opt.deg then
        d2r = %pi/180.0;
        theta = d2r*theta;
    end

    
	cth = cos(theta);
	sth = sin(theta);
	vth = (1 - cth);
    if norm(k)~=1  then k=k/norm(k);   // normalize 
    end
	kx = k(1); ky = k(2); kz = k(3);

        // from Paul's book, p. 28
	R = clean([
kx*kx*vth+cth      ky*kx*vth-kz*sth   kz*kx*vth+ky*sth;
kx*ky*vth+kz*sth   ky*ky*vth+cth      kz*ky*vth-kx*sth;
kx*kz*vth-ky*sth   ky*kz*vth+kx*sth   kz*kz*vth+cth
	]);
endfunction
