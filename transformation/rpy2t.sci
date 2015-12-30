// =============================================================================
// rpy2t.sci - Convert RPY angles to an homogeneous transformation
// www.controlsystemslab.com   July 2012
// =============================================================================

function T = rpy2tr (rpy,varargin)
// T = RPY2T(RPY, OPTIONS) is a homogeneous tranformation matrix equivalent to the 
// specified roll, pitch, yaw angles which correspond to rotations about the 
// X, Y, Z axes respectively. If RPY has multiple rows they are assumed to 
// represent a trajectory and T is a three dimensional matrix, where the last index  
// corresponds to the rows of RPY.
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

    R = rpy2r(rpy, varargin);
    T = r2t(R);

endfunction

// -----------------------------------------------------------------------------

rpy2t = rpy2tr

// =============================================================================
