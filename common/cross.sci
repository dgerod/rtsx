// cross.sci  vector cross product

function C = cross(A,B)
    if (length(A)==3 & length(B)==3) then
    
       C = [A(2)*B(3)-A(3)*B(2);
             A(3)*B(1)-A(1)*B(3);
            A(1)*B(2)-A(2)*B(1)];
     else
         error("argument must be vectors in R3");
     end
endfunction
