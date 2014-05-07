// angvec2t.sci  
// www.controlsystemslab.com  July 2012
//Convert angle and vector orientation to a homogeneous transform

// T = ANGVEC2T(THETA, K, options) is a homogeneous transform matrix equivalent to
// a  rotation of THETA about the vector K.
function T = angvec2t(theta, k, varargin)
    R = angvec2r(theta, k, varargin);
    T = r2t(R);   // convert R to T
endfunction