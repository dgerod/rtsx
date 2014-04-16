// AttachTool.sci   attach tool frame to robot
// www.controlsystemslab.com  August 2012

function robot=AttachTool(robot,Tt)
    if isequal(size(Tt),[4,4]) & isequal(Tt(4,:),[0 0 0 1]) then
        robot.tool = Tt;
    else
        disp(Tt);
        error("Bad data for Tt");    
    end
endfunction

function robot=attachtool(robot,Tt)
    robot=AttachTool(robot,Tt);
endfunction