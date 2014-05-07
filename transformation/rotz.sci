// ROTZ Rotation about Z axis
// www.controlsystemslab.com  July 2012
// R = rotz(theta) represents basic rotation matrix about coordinate axis Z

function R=rotz(th,varargin)
    opt.deg = 0;

    // process varargin 
    numopts=length(varargin); // find number of options
    if numopts>0 then
        for i=1:numopts
            if varargin(i)== 'deg' then opt.deg = 1;
            end
        end
    end
    // optionally convert from degrees
    if opt.deg then
        d2r = %pi/180.0;
        th = d2r*th;
    end    
    cth = cos(th);
    sth = sin(th);
    R = clean([cth -sth 0;sth cth 0;0 0 1]);
endfunction