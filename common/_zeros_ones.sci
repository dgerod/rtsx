// _zeros_ones.sci  my version of zeros(), ones(), and eye() 
//that works correctly for commands like _zeros(size(A))
// www.controlsystemslab.com   August 2012


function Z=_zeros(nr,nc)
    nargin = argn(2);
    
    select nargin
    case 0
        Z = 0;
    case 1
        if isequal(size(nr),[1 2]) then
            Z = zeros(nr(1),nr(2));
        else
            error("Argument must be a 1 x 2 vector");
        end
        
    case 2
        Z = zeros(nr,nc);
    else
        error("Invalid number of arguments");
    end
endfunction


function O=_ones(nr,nc)
    nargin = argn(2);
    
    select nargin
    case 0
        O = 1;
    case 1
        if isequal(size(nr),[1 2]) then
            O = ones(nr(1),nr(2));
        else
            error("Argument must be a 1 x 2 vector");
        end
        
    case 2
        O = ones(nr,nc);
    else
        error("Invalid number of arguments");
    end
endfunction

function I=_eye(nr,nc)
    nargin = argn(2);
    
    select nargin
    case 0
        I = 1;
    case 1
        if isequal(size(nr),[1 2]) then
            I = eye(nr(1),nr(2));
        else
            error("Argument must be a 1 x 2 vector");
        end
        
    case 2
        I = eye(nr,nc);
    else
        error("Invalid number of arguments");
    end
endfunction

