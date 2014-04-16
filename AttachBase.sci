// AttachBase.sci   attach base frame to robot
// www.controlsystemslab.com  August 2012

function robot=AttachBase(robot,Tb)
    if isequal(size(Tb),[4,4]) & isequal(Tb(4,:),[0 0 0 1]) then
        robot.base = Tb;
    else
        disp(Tb);
        error("Bad data for Tb");    
    end
endfunction

function robot=attachbase(robot,Tb)
    robot=AttachBase(robot,Tb);
endfunction