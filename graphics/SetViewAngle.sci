// SetViewAngle.sci   save viewangle of a figure to a robot
// www.controlsystemslab.com  July 2012

function robot=SetViewAngle(robot,winnum)
    if winnum~=winsid() then
        error("Invalid window number");
    else
        figure(winnum);
        h=gca();
        robot.viewangle=h.rotation_angles;
    end
endfunction

function robot=setviewangle(robot,winnum)
    robot=SetViewAngle(robot,winnum);
endfunction