// qadd.sci add quaternaion
// www.controlsystemslab.com  Sep 2012

function q = qadd(q1, q2)
       if isquaternion(q1)& isquaternion(q2) then    
          nq1 = size(q1,1);
          nq2 = size(q2,1);
          if nq1 == nq2 then
                   for i = 1:nq1
                       q(i).s = q1(i).s + q2(i).s;
                       q(i).v = q1(i).v + q2(i).v;
                   end
            else 
                error("Arguments do not have same number of elements")
          end
      else 
          error("Arguments must be quaternions");
      end
endfunction


function q = qsubtract(q1,q2)
    nq2 = size(q2,1);
    for i=1:nq2
    
       q2(i).s = -q2(i).s;
       q2(i).v = -q2(i).v;
    end
    q = qadd(q1, q2);
endfunction

