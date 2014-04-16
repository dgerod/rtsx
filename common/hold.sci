// hold.sci   mimic MATLAB hold on/off
// www.controlsystemslab.com  July 2012
// Usage: hold(0) = hold off
//        hold(1) = hold on

function []=hold(opt)
    if opt==0 then set(gca(),"auto_clear","on");
        else set(gca(),"auto_clear","off");
    end
endfunction