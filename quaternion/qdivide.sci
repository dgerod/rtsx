 function q = qdivide(q1, q2)
   //Quaternion.mrdivide Quaternion quotient.
   //
   // Q1/Q2   is a quaternion formed by Hamilton product of Q1 and inv(Q2).
   // Q/S     is the element-wise division of quaternion elements by the scalar S.
   //

       if isquaternion(q2)
           // qq = q1 / q2
           //    = q1 * qinv(q2)
   
           q = q1 * qinv(q2);
       elseif isscalar(q2)
           q.s = q1.s/q2; 
           q.v = q1.v/q2;
       end
endfunction
