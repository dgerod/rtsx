//RP2T.sci Convert rotation and translation to homogeneous transform
//
// rp2t() and rd2t() are equivalent. Choose the name you like


// adapted rom The Robotics Toolbox for MATLAB (RTB)
// Copyright (C) 1993-2011, by Peter I. Corke


function T = rp2t(R, p)
    seqdata = 0;  // sequence or not
    if numcols(R) ~= numrows(R)
        error('R must be square');
    end
    if numrows(R) ~= numrows(p)
        error('R and t must have the same number of rows');
    end
    d = size(R);
    if length(d)==3 then seqdata = 1;
    end
    if seqdata & d(3) ~= numcols(p)
        error('For sequence size(R,3) must equal size(t,2)');
    end

    if seqdata
        Z = zeros(numcols(R),1);
        B = [Z' 1];
        T = zeros(4,4,size(R,3));
        for i=1:size(R,3)
            T(:,:,i) = [R(:,:,i) p(:,i); B];
        end
    else
        T = [R p; zeros(1,numcols(R)) 1];
    end
endfunction

function T=rd2t(R,d)
    T=rp2t(R,d);
endfunction