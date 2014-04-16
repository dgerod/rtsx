// isquaterion.sci  check if input is a quaterion

function h=isquaternion(q)
    if type(q)==17 & ndims(q) == 2  then 
        if isfield(q,'s') & isfield(q, 'v') then   
            if length(q.s) == 1 then
            
               h = type(q.s)==1 & type(q.v)==1 & length(q.s)==1 & isequal(size(q.v),[1 3]); 
            else
                h = (1==1);  // true
                for i = 1:size(q,1)
                    
                    h = h & type(q(i).s)==1 & type(q(i).v)==1 & length(q(i).s)==1 & isequal(size(q(i).v),[1 3]); 
                end
             end
        else
            h = (1==0);  // return false 
        end
    else
        h = (1==0);  // return false
    end
    
endfunction

function h=IsQuaternion(q)
    h = isquaternaion(q)
endfunction    

