// qnorm.sci  Quaternion magnitude
// www.controlsystemslab.com  Sep 2012
//
 // QN = Q.norm(Q) is the scalar norm or magnitude of the quaternion Q.  
 //
 // Notes::
 // - This is the Euclidean norm of the quaternion written as a 4-vector.
 // - A unit-quaternion has a norm of one.

function qmag = qnorm(q)
    nq = size(q,1);
    if nq==1 then
        v = q2vec(q);
        qmag = norm(v); 

    else
        for i=1:nq
            v = q2vec(q(i));
            qmag(i) = norm(v);
        end
    end    
endfunction
