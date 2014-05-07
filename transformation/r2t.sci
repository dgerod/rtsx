// r2t.sci  augment rotation matrix R to homogeneous transformation T
// www.controlsystemslab.com   July 2012
// Note: this funciton can handle multi-dimensional case
function T=r2t(R)
    d = size(R);
    zv = zeros(3,1);
    B = [0 0 0 1];
    if length(d)==2 then
        T = [R(1:3,1:3) zv; B];
    else
        for i=1:d(3)
            T(:,:,i)=[R(1:3,1:3,i) zv; B];
        end
    end
endfunction
