// eul2r.sci make a rotation matrix from euler angles
// www.controlsystemslab.com  July 2012
// use rotz(), roty()
// 
// R = EUL2R(EUL, OPTIONS) is an orthonornal rotation matrix 
// equivalent to the specified Euler angles EUL = [PHI THETA PSI].
//  These correspond to rotations 
// about the Z, Y, Z axes respectively. If EUL is a matrix, 
// then it is assumed to represent a trajectory and R is a three dimensional 
// matrix, where the last index corresponds to rows of EUL
// Options::
//  'deg'      Compute angles in degrees (radians default)
// This file is adapted from The RObotics Toolbox for MATLAB (RTS)
// Copyright (C) 1993-2011, by Peter I. Corke


function R=eul2r(eul,varargin)
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
        eul = d2r*eul;
    end


    // unpack the arguments
    if numcols(eul)==3 then
    
        theta = eul(:,2);
	    psi = eul(:,3);
    	phi = eul(:,1);
    else
        error("Wrong size argument. Must be an n x 3 vector") 
    end

    if numrows(eul) == 1 then
            R = rotz(phi) * roty(theta) * rotz(psi);
    else
            R = zeros(3,3,numrows(eul));
            for i=1:numrows(eul)
                R(:,:,i) = rotz(phi(i)) * roty(theta(i)) * rotz(psi(i));
            end
    end

endfunction
