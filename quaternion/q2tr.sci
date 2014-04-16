//Q2TR   Convert unit-quaternion to homogeneous transform
//
//   [T,R] = q2tr(Q)
//
//   Return the rotational homogeneous transform corresponding to the unit
//   quaternion Q.
//
//   See also: TR2Q

function [t,r] = q2tr(q)

    q = q2vec(q);
    s = q(1);
    x = q(2);
    y = q(3);
    z = q(4);

    r = [   1-2*(y^2+z^2)   2*(x*y-s*z) 2*(x*z+s*y)
        2*(x*y+s*z) 1-2*(x^2+z^2)   2*(y*z-s*x)
        2*(x*z-s*y) 2*(y*z+s*x) 1-2*(x^2+y^2)   ];
    t = eye(4,4);
    t(1:3,1:3) = r;
    
endfunction

function R = q2r(q)    
    // converts quaternion to rotation matrix
    // this function accepts a sequence of quaternion
    nq = size(q,1);    // number of elements in q
    R = zeros(3,3,nq);
    for i=1:nq
        [T,R] = q2tr(q(i));
        R(:,:,i) = R;
    end
endfunction

function T = q2t(q)    
    // converts quaternion to homogeneous transformation matrix
    // this function accepts a sequence of quaternion
    nq = size(q,1);    // number of elements in q
    T = zeros(4,4,nq);
    for i=1:nq
        T(:,:,i) = q2tr(q(i));
    end
endfunction
