// MTRANSL.SCI  basic homogeneous transformation for translation
// 3-D matrix version
// www.controlsystemslab.com  July 2012
// translate a point or coordinate frame to new locations p
// where p could be a sequence of points
// Example:
// p = [2 3 -1]';
// T = transl(p);

function T=mtransl(p)
    d3 = 0;  // flag for 3-D matrix
    d = size(p);

    if  d(2)==1 then
        if d ~= [3 1] error("size of vector p must be 3x1 or 1x3");
        end
        T=[eye(3,3) p; 0 0 0 1];
    else 
        if d(1)~= 3 error("matrix p must have 3 rows"); end
        for i=1:d(2)
            T(:,:,i)=[eye(3,3) p(:,i); 0 0 0 1];
        end
     end       
endfunction