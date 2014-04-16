//TR2ANGVEC Convert rotation matrix to angle-vector form
//
// [THETA,V] = TR2ANGVEC(R) converts an orthonormal rotation matrix R into a 
// rotation of THETA (1x1) about the axis V (1x3).
//
// [THETA,V] = TR2ANGVEC(T) as above but uses the rotational part of the
// homogeneous transform T.
//

// Notes::
// - If no output arguments are specified the result is displayed.
// - This algorithm is from Paul 1981, other solutions are possible using
//   eigenvectors or Rodriguez formula.
//
// See also ANGVEC2R, ANGVEC2T.

// This function is ported from The RObotics Toolbox for MATLAB (RTS)
// Copyright (C) 1993-2011, by Peter I. Corke


function [theta,v]=tr2angvec(R, varargin)
    opt.deg = 0;

    // process varargin 
    numopts=length(varargin); // find number of options
    if numopts>0 then
        for i=1:numopts
            if varargin(i)== 'deg' then opt.deg = 1;
            end
        end
    end
    if ~isrot(R)
        R = t2r(R);
    end
    d=size(R);
    if length(d)==3 // size(R,3) > 1
        
        theta = zeros(d(3),1);
        v = zeros(d(3),3);
        for i=1:d(3)
            [t,a] = tr2angvec(R(:,:,i));
            theta(i) = t;
            v(i,:) = a;
        end
        return
    end
    
	qs = sqrt(trace(R)+1)/2.0;
    qs = min(qs, 1);
        
	kx = R(3,2) - R(2,3);	// Oz - Ay
	ky = R(1,3) - R(3,1);	// Ax - Nz
	kz = R(2,1) - R(1,2);	// Ny - Ox

	if (R(1,1) >= R(2,2)) & (R(1,1) >= R(3,3)) 
		kx1 = R(1,1) - R(2,2) - R(3,3) + 1;	// Nx - Oy - Az + 1
		ky1 = R(2,1) + R(1,2);			// Ny + Ox
		kz1 = R(3,1) + R(1,3);			// Nz + Ax
		add = (kx >= 0);
	elseif (R(2,2) >= R(3,3))
		kx1 = R(2,1) + R(1,2);			// Ny + Ox
		ky1 = R(2,2) - R(1,1) - R(3,3) + 1;	// Oy - Nx - Az + 1
		kz1 = R(3,2) + R(2,3);			// Oz + Ay
		add = (ky >= 0);
	else
		kx1 = R(3,1) + R(1,3);			// Nz + Ax
		ky1 = R(3,2) + R(2,3);			// Oz + Ay
		kz1 = R(3,3) - R(1,1) - R(2,2) + 1;	// Az - Nx - Oy + 1
		add = (kz >= 0);
	end

	if add
		kx = kx + kx1;
		ky = ky + ky1;
		kz = kz + kz1;
	else
		kx = kx - kx1;
		ky = ky - ky1;
		kz = kz - kz1;
	end
	v = unit([kx ky kz]);
    theta = 2*acos(qs);
    if theta > %pi
        theta = %pi - theta;
        v = -v;
    end
    v = v';   // convert to 3x1 vector 
    if opt.deg then
        r2d = 180/%pi;
        theta = r2d*theta;
    end
    if argn(1) == 0
        fprintf('Rotation: %f rad x [%f %f %f]\n', theta, v(1), v(2), v(3));
    end
endfunction