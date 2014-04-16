// SerialFrame.sci  construct a chain of coordinate frame
// computing missing information if possible
// www.controlsystemslab.com   August 2012

function sf=SerialFrame(F,varargin)
    sf=_Serial_Frame(F,varargin);
endfunction

function sf=serialframe(F,varargin)
    sf = _Serial_Frame(F,varargin);
endfunction

function sf=_Serial_Frame(F,varargin)
    varargin = varargin($);
    varnum=length(varargin); 
    printf("\nReading frame data and computing missing information\n");    
    nframes = size(F,1);
    sf.nf = nframes;  // keep number of frames
    sf.name = '';

    CompleteCount = 0;    // count frames that are completed
    AboveMissing = 0;    // flag to indicate frames created above missing frame

    sf.missingframe = 0;  // keep missing frame
    sf.viewangle = [];   // view angle to be used by PlotFrame
    sf.completed = 0;
    nullcount = 0;   // counter for frame that have Tabs and Trel both null
    
    // ==========process variable arguments ================
     if pmodulo(varnum,2) then  // number of arguments is odd. Error!
          error("*** Variable input argument number is odd. You must miss something! ***");
     else
        for iv =1:2:varnum-1  // select only command at odd position
            if type(varargin(iv))==10 then  // check if string (it should be!)
                varargin(iv)=convstr(varargin(iv),'l'); // convert to lower 
            else
                error("*** Parameter name must be a string (perhaps you forget to put it in quotes)! ***");                
            end
            select varargin(iv),
                case 'name' then 
                    if  type(varargin(iv+1))==10 then
                         sf.name = varargin(iv+1);
                    else error("Frame name must be a string");
                    end
                case 'viewangle' then
                    if size(varargin(iv+1))==[1,2] then                       
                        sf.viewangle = varargin(iv+1);
                    else error("*** Value for frame viewangle must be 1 x 2 vector ***");
                    end

                end // select varargin(iv)
            end // for iv=1:2:varnum-1
        end // if pmodulo(varnum,2)  
            
    printf("\nProcessing Upwards ...\n\n");
    for i=1:nframes
        sf.Frame(i)=F(i);

        if F(i).name == '' then    // set name to frame i
            sbuf = sprintf("%d",i);
            sf.Frame(i).name = sbuf;
        else sf.Frame(i).name = F(i).name;
        end
        // solve for missing information
        
       

        if i==1 then // for base frame, simply set Tabs = Trel
            if sf.Frame(i).Tabs == [] & sf.Frame(i).Trel == []
                // base frame must have either Tabs or Trel information
                error("No information for base frame");
             elseif  sf.Frame(i).Tabs == [];
                 sf.Frame(i).Tabs = sf.Frame(i).Trel;
                 printf("%d -- {%s} : Found T_rel. Fill in T_abs with T_rel\n",i,sf.Frame(i).name);
             elseif sf.Frame(i).Trel == [];
                 sf.Frame(i).Trel = sf.Frame(i).Tabs;
                 printf("%d -- {%s} : Found T_abs. Fill in T_rel with T_abs\n",i,sf.Frame(i).name);                 
             end
             sf.Frame(i).completed = 1;  // frame 1 completed
             CompleteCount = 1;
 
         else  // i not equal to 1 

              if ~AboveMissing & sf.Frame(i).Tabs == [] & sf.Frame(i).Trel~=[] then
                 // know only Trel, compute Tabs

                 printf("%d -- {%s}: Found T_rel. Computing T_abs :{%s} w.r.t {%s}",i,sf.Frame(i).name, sf.Frame(i).name, sf.Frame(1).name);                  
                 sf.Frame(i).Tabs = sf.Frame(i-1).Tabs*sf.Frame(i).Trel;
                 printf("-- Finished\n");
                 sf.Frame(i).completed = 1;
                 CompleteCount = CompleteCount+1;
             elseif ~AboveMissing & sf.Frame(i).Tabs ~= [] & sf.Frame(i).Trel==[] then
                 // know only Tabs, compute Trel
                 printf("%d -- {%s}: Found T_abs. Computing T_rel :{%s} w.r.t {%s}",i,sf.Frame(i).name, sf.Frame(i).name, sf.Frame(i-1).name);
                 sf.Frame(i).Trel = inv(sf.Frame(i-1).Tabs)*sf.Frame(i).Tabs; 
                 printf("-- Finished\n");
                 sf.Frame(i).completed = 1;
                 CompleteCount = CompleteCount+1;
            elseif sf.Frame(i).Tabs == [] & sf.Frame(i).Trel == [] then //  both Tabs & Trel null
                printf("%d -- {%s}: *** Missing both T_abs: {%s} w.r.t {%s} and T_rel: {%s} w.r.t {%s} ***\n",i,sf.Frame(i).name, sf.Frame(i).name, sf.Frame(1).name, sf.Frame(i).name, sf.Frame(i-1).name);
                sf.Frame(i).completed = 0;
                sf.missingframe = i;
                AboveMissing = 1;    // all frames above this one must be given T_rel only
                nullcount = nullcount+1;
                if nullcount == 2

                    error("*** Both T_abs and T_rel can be left null for only 1 frame. ***");
                end 

            elseif AboveMissing & sf.Frame(i).Trel == []
                printf("%d -- {%s}: *** Missing T_rel: {%s} w.r.t {%s} ***\n\n",i,sf.Frame(i).name, sf.Frame(i).name, sf.Frame(i-1).name);
                printf("*** %s is unsolvable! ***\n\n",sf.name)
                 error("*** T_rel for any frame above missing frame cannot be left blank ***");
            elseif AboveMissing & sf.Frame(i).Tabs == []
                printf("%d -- {%s}: *** Missing T_abs: {%s} w.r.t {%s} ***\n",i,sf.Frame(i).name, sf.Frame(i).name, sf.Frame(1).name);
                sf.Frame(i).completed = 0; 
                         
            end  // if ~AboveMissing
         end   // if i== 1    
    end        // for i=1:nframes
    


        // =============== print summary ==================
        if CompleteCount == nframes then
            sf.completed = 1;
            printf("\n%s: All missing information are computed and filled in\n",sf.name);
        else
            sf.completed = 0;
            printf("\nList of missing frames in %s",sf.name);
            printf("\n===============================\n");
            
            for i=1:nframes
                
                if sf.Frame(i).completed == 0 then
                    printf("\n%d -- {%s}: ",i, sf.Frame(i).name);
                    missabs = 0;
                    if sf.Frame(i).Tabs == [] then
                        printf("T_abs : {%s} w.r.t {%s}", sf.Frame(i).name, sf.Frame(1).name);
                        missabs = 1;
                    end
                    if sf.Frame(i).Trel == [] then
                        if missabs==1,  // add a colon
                            printf(" , ");
                        end
                        printf("T_rel : {%s} w.r.t {%s}", sf.Frame(i).name, sf.Frame(i-1).name);
                    end   
                end  // sf.Frame(i).completed = 0
            end  // for i=1:nframes
            printf("\n");

        end      // if CompleteCount == nframes    
        //pause;   
endfunction
