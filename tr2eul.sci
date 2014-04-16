// tr2eul.sci find euler angles from homogeneous transformation matrix T
// ()or rotation matrix R)
// www.controlsystemslab.com  July 2012

//  EUL = TR2EUL(T, OPTIONS) are the ZYZ Euler angles expressed as a row vector
// corresponding to the rotational part of a homogeneous transform T.
// The 3 angles EUL=[PHI,THETA,PSI] correspond to sequential rotations about 
// the Z, Y and Z axes respectively.
//
// EUL = TR2EUL(R, OPTIONS) are the ZYZ Euler angles expressed as a row vector
// corresponding to the orthonormal rotation matrix R.
//
// If R or T represents a trajectory (has 3 dimensions), then each row of EUL
// corresponds to a step of the trajectory.
//
// Options::
//  'deg'      Compute angles in degrees (radians default)
//
// Notes::
// - There is a singularity for the case where THETA=0 in which case PHI is arbitrarily
//   set to zero and PSI is the sum (PHI+PSI).
// This function is ported from The RObotics Toolbox for MATLAB (RTS)
// Copyright (C) 1993-2011, by Peter I. Corke

//
function eul=tr2eul(T,varargin)

    opt.deg = 0;

    // process varargin 
    numopts=length(varargin); // find number of options
    if numopts>0 then
        for i=1:numopts
            if varargin(i)== 'deg' then opt.deg = 1;
            end
        end
    end
    d = size(T);
    if length(d)>2 then d3 = d(3);
    else d3 = 1;
    end
    eul = [];

    for i=1:d3, 
        if d3 == 1 then Ti = T;
        else Ti=T(:,:,i);
        end
        eul1 = zeros(1,3);
       
	    // Method as per Paul, p 69.
	    // phi = atan2(ay, ax)
	    // Only positive phi is returned.
	    if abs(Ti(1,3)) < %eps & abs(Ti(2,3)) < %eps // singularity
		    eul1(1) = 0;
		    sp = 0;
	    	cp = 1;
		    eul1(2) = atan(cp*Ti(1,3) + sp*Ti(2,3), Ti(3,3));
		    eul1(3) = atan(-sp*Ti(1,1) + cp*Ti(2,1), -sp*Ti(1,2) + cp*Ti(2,2));
	    else
    		eul1(1) = atan(Ti(2,3), Ti(1,3));
    		sp = sin(eul1(1));
	    	cp = cos(eul1(1));
	    	eul1(2) = atan(cp*Ti(1,3) + sp*Ti(2,3), Ti(3,3));
	    	eul1(3) = atan(-sp*Ti(1,1) + cp*Ti(2,1), -sp*Ti(1,2) + cp*Ti(2,2));
	    end

        if opt.deg == 1
            eul1 = eul1 * 180/%pi;
        end 
        eul = [eul; eul1]   
    end
endfunction