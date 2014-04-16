// RPY2T.SCI  convert Row-Pitch-Yaw angles to homogeneous transformation
// www.controlsystemslab.com   July 2012
// T = RPY2T(RPY, OPTIONS) is a homogeneous tranformation matrix equivalent to the 
// specified roll, pitch, yaw angles which correspond to rotations about the 
// X, Y, Z axes respectively. If RPY has multiple rows they are assumed to 
// represent a trajectory and T is a three dimensional matrix, where the last index  
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
function T=rpy2t(rpy, varargin)
    R=rpy2r(rpy,varargin);
    T=r2t(R);
endfunction