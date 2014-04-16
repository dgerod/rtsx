// polyval.sci   evaluate polynomial 
// www.controlsystemslab.com  August 2012

function Y=polyval(P,X)
    Y = 0*X;
    n = length(P);
    for i=1:n
        Y = Y + P(i)*X.^(n-i);
    end    
    
endfunction
