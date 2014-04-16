// eul2t.sci make a homogeneous transformation matrix from euler angles
// www.controlsystemslab.com  July 2012
// use eul2r(),r2t()
// Example:
// eul = [%pi/2 %pi/3 %pi/4];
// T = eul2t(eul);

function T=eul2t(eul,varargin)
    R = eul2r(eul,varargin);
    T = r2t(R);
endfunction