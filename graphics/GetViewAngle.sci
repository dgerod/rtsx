// GetViewAngle.sci  get viewangle of current figure

function vangle=GetViewAngle(winnum)
    if winnum~=winsid() then
        error("Invalid window number");
    else
        figure(winnum);
        h=gca();
        vangle=h.rotation_angles;
    end    
endfunction
