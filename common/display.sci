// display.sci   display data
// www.controlsystemslab.com  Sep 2012

function display(a)
    if isquaternion(a) then
        s = q2str(a);  // convert to string first
        ns = size(s,1); 
        if ns==1 then
            printf("\nQuaternion data\n");
            printf(s);
        else
            printf("\n");
            printf("======= A quaternion array of %d elements =======\n", ns)
           for i = 1:ns
               printf("%d -- %s\n",i,s(i)); 
           end
        end
     else
         printf("So far only quaternion display is implemented");
    end
endfunction
