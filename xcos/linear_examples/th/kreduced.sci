//kreduced.sci 
// perform controller reduction using balanced truncation method

function Kr=kreduced(K, nk)
    if argn(2)==1  // no nk passed
        nk = 0;
    end
    [Kb,U]=balreal(K);

    hsv=hankelsv(Kb);
    if nk == 0,
       disp(hsv);        
       nk=input('Enter number of states to keep: ');
    end
    [A,B,C,D]=abcd(Kb);
    At = A(1:nk,1:nk);
    Bt = B(1:nk,:);
    Ct = C(:,1:nk);
    Kr=syslin('c',At,Bt,Ct,D);
endfunction
