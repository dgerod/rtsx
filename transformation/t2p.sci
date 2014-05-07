// T2P.SCI  extract translation part from homogenous transformation matrix
// www.controlsystemslab.com  July 2012
// t2d() and t2p() are equivalent. Choose the function name that you like

function d=t2p(T)
    td = size(T);
    if length(td)==2 then
        d = T(1:3,4);
    else
        d = zeros(3,td(3))
        for i=1:td(3)
            d(:,i) = T(1:3,4,i);
        end
    end
    
endfunction

function d=t2d(T)
    d=t2p(T);
endfunction