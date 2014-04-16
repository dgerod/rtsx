// T2R.SCI  extract rotation part from homogenous transformation matrix
// www.controlsystemslab.com  July 2012
//

function R=t2r(T)
    d = size(T);
    if length(d)==2 then
        R = T(1:3,1:3);
    else
        for i=1:d(3)
            R(:,:,i)= T(1:3,1:3,i);
        end
    end
endfunction