// ResolveFrame.sci  Find missing frame information in a closed loop kinematic
// chain
// www.controlsystemslab.com   August 2012
// kc1 is the frame to be solved
// kc2 must be a completed frame, or just a homogeneous matrix describing
// the end frame of kc1 w.r.t base
// return resolved kc1 as well as T_rel and T_abs of missing frame

function [T_rel,T_abs,kc1]=ResolveFrame(kc1,kc2)
    printf("\nRunning ResolveFrame ...\n");
    nframes1 = size(kc1.Frame,1);
    if type(kc2)== 17 then   
            nframes2 = size(kc2.Frame,1);
            T = kc2.Frame(nframes2).Tabs;
            if T==[] then
                msg=sprintf("%s is unsolvable due to missing information in %s",kc1.name,kc2.name);
                error(msg);
            end
    elseif ishomog(kc2,'valid')
            T = kc2;
    else
            error("Bad 2nd input argument")
    end    // if type(kc2)== 17
    
    if kc1.completed 
            printf("%s is already a complete kinematic chain. Nothing left to be solved\n",kc1.name);
    else   
            
             
            missingframe = kc1.missingframe;
            if missingframe < nframes1,
                if kc1.Frame(nframes1).Trel ~= [] then
                    printf("\n%d -- {%s}: Fill in T_abs : {%s} w.r.t {%s}",nframes1,kc1.Frame(nframes1).name, kc1.Frame(nframes1).name, kc1.Frame(1).name);
                    kc1.Frame(nframes1).Tabs = T;                
                    kc1.Frame(nframes1).completed = 1;
                    printf("-- Finished \n");
                else
    
                    printf("%d -- {%s}: *** missing T_rel: {%s} w.r.t {%s} ***",nframes1,kc1.Frame(nframes1).name,kc1.Frame(nframes1).name, kc1.Frame(nframes1-1).name);
                    msg=sprintf("Cannot resolve %s",kc1.name);
                    error(msg);
                    
                end    // if kc1.Frame(nframes).Trel ~= []
            end    // if missingframe < nframes1
            if nframes1-missingframe>1
                for i=nframes1-1:-1:missingframe+1
                
                    if kc1.Frame(i+1).Trel ~= [] then
                
    
                        printf("%d -- {%s}: Computing T_abs : {%s} w.r.t {%s} ",i,kc1.Frame(i).name,kc1.Frame(i).name, kc1.Frame(1).name);
                        kc1.Frame(i).Tabs = inv(kc1.Frame(i+1).Trel)*kc1.Frame(i+1).Tabs;      
                        printf("-- Finished \n");
                        kc1.Frame(i).completed = 1;          
                    else
                        printf("%d -- {%s}: *** missing T_rel: {%s} w.r.t {%s} ***",i,kc1.Frame(i).name,kc1.Frame(i).name, kc1.Frame(i-1).name);
                        msg=sprintf("Cannot resolve %s",kc1.name);
                        error(msg);
                     end  // if kc1.Frame(i+1).Trel ~= []
                 end // for i=nframes1-1:
             end 
             // now solve for missing frame
             // starting with T_abs
             if missingframe < nframes1
                 printf("%d -- {%s} (Missing Frame): Computing T_abs : {%s} w.r.t {%s} ",missingframe,kc1.Frame(missingframe).name,kc1.Frame(missingframe).name, kc1.Frame(1).name);            
                 kc1.Frame(missingframe).Tabs = inv(kc1.Frame(missingframe+1).Trel)*kc1.Frame(missingframe+1).Tabs;
                T_abs = clean(kc1.Frame(missingframe).Tabs);
                 printf("-- Finished \n");
             else
                 printf("%d -- {%s} (Missing Frame): fill in T_abs : {%s} w.r.t {%s} ",missingframe,kc1.Frame(missingframe).name,kc1.Frame(missingframe).name, kc1.Frame(1).name);            
                 kc1.Frame(missingframe).Tabs = T;
                 printf("-- Finished \n");                 
                 T_abs = clean(T);                 
             end    // if missingframe < nframes1

             // computing T_rel
             printf("%d -- {%s} (Missing Frame): Computing T_rel : {%s} w.r.t {%s} ",missingframe,kc1.Frame(missingframe).name,kc1.Frame(missingframe).name, kc1.Frame(missingframe-1).name); 

             if kc1.Frame(missingframe-1).Tabs ~= []           
                 kc1.Frame(missingframe).Trel = inv(kc1.Frame(missingframe-1).Tabs)*kc1.Frame(missingframe).Tabs;
                T_rel = clean(kc1.Frame(missingframe).Trel);
                 printf("-- Finished \n"); 
                 kc1.Frame(missingframe).completed = 1;
                 kc1.missingframe =0;    // reset missing frame number
              else // missing T_abs of frame below
                    printf("%d -- {%s} (Missing Frame): *** missing T_abs: {%s} w.r.t {%s} ***",missingframe,kc1.Frame(missinframe).name,kc1.Frame(missingframe-1).name, kc1.Frame(1).name);
                    msg=sprintf("Cannot resolve %s",kc1.name);
                    error(msg);  
               end  // if kc1.Frame(missingframe-1).Tabs ~= []                        

        end // if kc1.completed
    
    
    
        // checking if the chain is completed
        printf("\nRechecking if there are still missing frame data in %s",kc1.name);
        CompleteFlag = 1;
        for i=1:nframes1
            if kc1.Frame(i).completed == 0 then
                printf("\n%d -- %s: ",i, kc1.Frame(i).name);
                if kc1.Frame(i).Tabs == [] then
                    printf("Still missing T_abs : {%s} w.r.t {%s} , ", kc1.Frame(i).name, kc1.Frame(1).name);
                    CompleteFlag = 0;
                end
                if kc1.Frame(i).Trel == [] then
                    printf("Still missing T_rel : {%s} w.r.t {%s} , ", kc1.Frame(i).name, kc1.Frame(i-1).name);
                    CompleteFlag = 0;
                end   
            end  // kc1.Frame(i).completed = 0
        end  // for i=1:nframes1
        if CompleteFlag,
            printf("-- none found.");
            printf("\n\n--- %s is now completed. ---\n",kc1.name);
            kc1.completed = 1;    // set completed flag
        else
            printf("\n%s is still incomplete\n",kc1.name);
        end       
       
endfunction

function [T_rel,T_abs,kc1]=resolveframe(kc1,kc2)
    [T_rel,T_abs,kc1]=ResolveFrame(kc1,kc2);
endfunction