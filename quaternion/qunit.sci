// qunit.sci  Unitize a quaternion

function qu= qunit(q)
    nq = size(q,1);
    if nq==1 then
        mq = qnorm(q);
        qu.s = q.s/mq;
        qu.v = q.v/mq; 

   else
         mq = qnorm(q);
        for i=1:nq
            qu(i).s = q(i).s/mq(i);
            qu(i).v = q(i).v/mq(i);
           
        end
    end      
endfunction
