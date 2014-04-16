// qinv.sci invert a unjit-quaternion
// www.controlsystemslab.com  Sep 2012

function qi = qinv(q)
    nq = size(q,1);
    if nq==1 then
        qi.s = q.s;
        qi.v = -q.v;
    else
        for i=1:nq
            qi(i).s = q(i).s;
            qi(i).v = -q(i).v;
        end
    end
endfunction
