// blkdiag2.sci   form a block diagonal matrix from 2 matrices

function Z = blkdiag2(X,Y)
    sizex = size(X);
    sizey = size(Y);
    Z = [X zeros(sizex(1),sizey(2)); zeros(sizey(1),sizex(2)) Y];
    
endfunction