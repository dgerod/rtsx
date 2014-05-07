// ROTX Rotation about X axis
// www.controlsystemslab.com  July 2012
// R = rotx(theta) represents basic rotation matrix about coordinate axis X

function R=rotx(th, varargin)
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
    R = clean([1 0 0; 0 cth -sth;0 sth cth]);
endfunction