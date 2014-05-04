// =====================================================================
// DetachTool.sci   detach tool frame from robot
// www.controlsystemslab.com  August 2012
// =====================================================================

function [robot,Tt] = DetachTool (robot)
    Tt = robot.tool;
    robot.tool = eye(4,4);
endfunction

// ---------------------------------------------------------------------

detachtool = DetachTool;

// =====================================================================
