// rpy2r.sci convert roll-pitch-yaw angles to rotation matrix R
// www.controlsystemslab.com  July 2012
//
// Note: this function is adapted from rpy2r() function from
//  The Robotics Toolbox for Matlab (RTB). 
// Copyright (C) 1993-2011, by Peter I. Corke

// R = RPY2R(RPY, OPTIONS) is an orthonormal rotation matrix equivalent to the 
// specified roll, pitch, yaw angles which correspond to rotations about the 
// X, Y, Z axes respectively. If RPY has multiple rows they are assumed to 
// represent a trajectory and R is a three dimensional matrix, where the last index  
// corresponds to the rows of RPY.
//

//
// Options::
//  'deg'   Compute angles in degrees (radians default)
//  'zyx'   Return solution for sequential rotations about Z, Y, X axes (Paul book)
//
// Note::
// - In previous releases (<8) the angles corresponded to rotations about ZYX. Many 
//   texts (Paul, Spong) use the rotation order ZYX. This old behaviour can be enabled 
//   by passing the option 'zyx'
//
// See also TR2RPY, EUL2TR.

function R = rpy2r(rpy, varargin)
    opt.zyx = 0;
    opt.deg = 0;
    
    // process varargin 
    numopts=length(varargin); // find number of options
    if numopts>0 then
        for i=1:numopts
            if varargin(i)== 'zyx' then opt.zyx = 1;
                elseif varargin(i)== 'deg' then opt.deg = 1;
            end
        end
    end

    // unpack the arguments
    if numcols(rpy) == 3
		pitch = rpy(:,2);
		yaw = rpy(:,3);
		roll = rpy(:,1);
    else
        error('bad argument size')
    end

    // optionally convert from degrees
    if opt.deg
        d2r = %pi/180.0;
        roll = roll * d2r;
        pitch = pitch * d2r;
        yaw = yaw * d2r;
    end

    if ~opt.zyx
        // XYZ order
        if numrows(roll) == 1
            R = rotx(roll) * roty(pitch) * rotz(yaw);
        else
            R = zeros(3,3,numrows(roll));
            for i=1:numrows(roll)
                R(:,:,i) = rotx(roll(i)) * roty(pitch(i)) * rotz(yaw(i));
            end
        end
    else
        // old ZYX order (as per Paul book)
        if numrows(roll) == 1
            R = rotz(roll) * roty(pitch) * rotx(yaw);
        else
            R = zeros(3,3,numrows(roll));
            for i=1:numrows(roll)
                R(:,:,i) = rotz(roll(i)) * roty(pitch(i)) * rotx(yaw(i));
            end
        end
    end
endfunction