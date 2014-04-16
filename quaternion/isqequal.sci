// isqequal.sci test quaterion equality
// Q1==Q2 is true if the quaternions Q1 and Q2 are equal.
//
// Notes::

// - Note that for unit Quaternions Q and -Q are the equivalent
//   rotation, so non-equality does not mean rotations are not
//   equivalent.
// - If Q1 is a vector of quaternions, each element is compared to 
//   Q2 and the result is a logical array of the same length as Q1.
// - If Q2 is a vector of quaternions, each element is compared to 
//   Q1 and the result is a logical array of the same length as Q2.
// - If Q1 and Q2 are vectors of the same length, then the result 
//   is a logical array

function e=isqequal(q1,q2)
    if isquaternion(q1)& isquaternion(q2) then
    
       nq1 = size(q1,1);
       nq2 = size(q2,1);
          if (nq1==1) & (nq2 == 1)
             e = isequal(q2vec(q1),q2vec(q2));
         elseif (nq1 >  1) & (nq2 == 1)
              v = q2vec(q2);
             for i=1:nq1
                 
                 e(i) = isequal(q2vec(q1(i)),v);
             end
         elseif (nq1 == 1) & (nq2 > 1)
              v = q2vec(q1);
             for i=1:nq2
                 e(i) = isequal(v,q2vec(q2(i)));
             end
         elseif nq1 == nq2
             
             for i=1:nq1
                 e(i) = isequal(q2vec(q1(i)),q2vec(q2(i)));
             end
         else
             error('Bad arguments');
         end
   else
       error("Input(s) is/are not quaternion");
   end
    
endfunction



