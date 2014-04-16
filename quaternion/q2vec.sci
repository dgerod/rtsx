// q2vec.sci 
//Quaternion.double Convert a quaternion to a 4-element vector
//
// V = Q.double() is a 4-vector comprising the quaternion
// elements [s vx vy vz].

function v = q2vec(q)
    nq = size(q,1);
    if nq==1 then
            v = [q.s q.v];
    else

        for i=1:nq
           v(i,:) = [q(i).s q(i).v];
        end
    end      



endfunction



