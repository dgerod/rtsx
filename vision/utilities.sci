// utilities.sci  
// utility functions for machine vision toolbox


//E2H Euclidean to homogeneous
//
// H = E2H(E) is the homogeneous representation of a set of points E.
//
// In the Toolbox points are represented as by Euclidean coordinates which are 
// the columns of a matrix E, and the number of rows is either 2 or 3 to 
// represent 2- or 3-dimensional points.  Homogeous representation increases
// the dimension by one
//
function h = e2h(e)
    h = [e; ones(1,numcols(e))];
endfunction

//H2E Homogeneous to Euclidean 
//
// E = H2E(H) is the Euclidean representation of a set of homogeneous points H.
//
// In the Toolbox points are represented as by Euclidean coordinates which are 
// the columns of a matrix E, and the number of rows is either 2 or 3 to 
// represent 2- or 3-dimensional points.  Euclidean representation decreases
// the dimension of each point by one.
//
// See also E2H

function e = h2e(h)

    if isvec(h)
        h = h(:);
    end
    e = h(1:numrows(h)-1,:) ./ repmat(h(numrows(h),:), numrows(h)-1, 1);

endfunction

//%HOMTRANS Apply a homogeneous transformation
//%
//% P2 = HOMTRANS(T, P) applies homogeneous transformation T to the points 
//% stored columnwise in P.
//%
//% - If T is in SE(2) (3x3) and
//%   - P is 2xN (2D points) they are considered Euclidean (R^2)
//%   - P is 3xN (2D points) they are considered projective (P^2)
//% - If T is in SE(3) (4x4) and
//%   - P is 3xN (3D points) they are considered Euclidean (R^3)
//%   - P is 4xN (3D points) they are considered projective (P^3)
//%
//%  TP = HOMTRANS(T, T1) applies homogeneous transformation T to the 
//%  homogeneous transformation T1, that is TP=T*T1.  If T1 is a 3-dimensional 
//%  transformation then T is applied to each plane as defined by the first two 
//% dimensions, ie. if T = NxN and T=NxNxP then the result is NxNxP.
//%
//% See also E2H, H2E.


function pt = homtrans(T, p)

    if numrows(p) == numrows(T)
        if ndims(p) == 3
            pt = [];
            for i=1:size(p,3)
                pt = cat(3, pt, T*p(:,:,i));
            end
        else
            pt = T * p;
        end
    elseif (numrows(T)-numrows(p)) == 1
        //% second argument is Euclidean coordinate, promote to homogeneous
        pt = h2e( T * e2h(p) );
    else
        error('matrices and point data do not conform')
    end
endfunction
