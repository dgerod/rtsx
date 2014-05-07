// TR2RPY Convert a homogeneous transform to roll-pitch-yaw angles
//
// RPY = TR2RPY(T, options) are the roll-pitch-yaw angles expressed as a row 
// vector corresponding to the rotation part of a homogeneous transform T.
// The 3 angles RPY=[R,P,Y] correspond to sequential rotations about 
// the X, Y and Z axes respectively.
//
// RPY = TR2RPY(R, options) are the roll-pitch-yaw angles expressed as a row 
// vector corresponding to the orthonormal rotation matrix R.
//
// If R or T represents a trajectory (has 3 dimensions), then each row of RPY
// corresponds to a step of the trajectory.
//
// Options::
//  'deg'   Compute angles in degrees (radians default)
//  'zyx'   Return solution for sequential rotations about Z, Y, X axes (Paul book)
//
// Notes::
// - There is a singularity for the case where P=pi/2 in which case R is arbitrarily
//   set to zero and Y is the sum (R+Y).
// - Note that textbooks (Paul, Spong) use the rotation order ZYX.
//
// See also  rpy2tr, tr2eul.
// This file is adapted from The RObotics Toolbox for MATLAB (RTS)
// Copyright (C) 1993-2011, by Peter I. Corke
function rpy=tr2rpy(T,varargin)
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
    d = size(T);
    if length(d)>2 then d3 = d(3);
    else d3 = 1;
    end
    rpy = [];

    for i=1:d3, 
        if d3 == 1 then m = T;
        else m=T(:,:,i);
        end
        rpy1 = zeros(1,3);
        if ~opt.zyx
            // XYZ order
            if abs(m(3,3)) < %eps & abs(m(2,3)) < %eps
                // singularity
                rpy1(1) = 0;  // roll is zero
                rpy1(2) = atan(m(1,3), m(3,3));  // pitch
                rpy1(3) = atan(m(2,1), m(2,2));  // yaw is sum of roll+yaw
            else
                rpy1(1) = atan(-m(2,3), m(3,3));        // roll
                // compute sin/cos of roll angle
                sr = sin(rpy1(1));
                cr = cos(rpy1(1));
                rpy1(2) = atan(m(1,3), cr * m(3,3) - sr * m(2,3));  // pitch
                rpy1(3) = atan(-m(1,2), m(1,1));        // yaw
            end
        else
            // old ZYX order (as per Paul book)
            if abs(m(1,1)) < %eps & abs(m(2,1)) < %eps
                // singularity
                rpy1(1) = 0;     // roll is zero
                rpy1(2) = atan(-m(3,1), m(1,1));  // pitch
                rpy1(3) = atan(-m(2,3), m(2,2));  // yaw is difference yaw-roll
            else
                rpy1(1) = atan(m(2,1), m(1,1));
                sp = sin(rpy1(1));
                cp = cos(rpy1(1));
                rpy1(2) = atan(-m(3,1), cp * m(1,1) + sp * m(2,1));
                rpy1(3) = atan(sp * m(1,3) - cp * m(2,3), cp*m(2,2) - sp*m(1,2));
            end
        end
        if opt.deg
            rpy1 = rpy1 * 180/%pi;
        end
        rpy = [rpy; rpy1]   
    end
endfunction