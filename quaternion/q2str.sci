// q2str.sci  convert quaternion to string
// www.controlsystemslab.com  Sep 2012
// S = Q.char() is a compact string representation of the quaternion's value
// as a 4-tuple.  If Q is a vector then S has one line per element.

function s=q2str(q)
    nq = size(q,1);
   if size(q,1) > 1   // a sequence of quaternion
      for i=1:nq;
          v = q(i).v;
         s(i) = sprintf("%f, < %f, %f, %f>",q(i).s, v(1),v(2), v(3));
       end
   else
       
       s = sprintf("%f, < %f, %f, %f>",q.s, q.v(1),q.v(2), q.v(3));
   end

endfunction
