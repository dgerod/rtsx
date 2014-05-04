// =====================================================================
// DetachBase.sci   detach base frame from robot
// www.controlsystemslab.com  August 2012
// =====================================================================

function [robot,Tb] = DetachBase (robot)
    Tb = robot.base;
    robot.base = eye(4,4);
endfunction

// ---------------------------------------------------------------------

detachbase = DetachBase;

// =====================================================================
