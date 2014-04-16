//ReplaceLink.sci  replace a robot link specified by li 
// www.controlsystemslab.com   August 2012
// usage: robot=ReplaceLink(robot,L,li);
// Example: 
// --> robot = ReplaceLink(robot,L, 2) // remove link 2 with L

function robot=ReplaceLink(robot,L,li)
    if argn(2)==0 then
       ReplaceLinkHelp();
        robot=[];
    else
        robot = _Replace_Link(robot,L,li);
    end          
endfunction

function robot=replacelink(robot,L,li)
    if argn(2)==0 then
       ReplaceLinkHelp();
        robot=[];
    else
        robot = _Replace_Link(robot,L,li);
    end 
endfunction
    
function robot=_Replace_Link(robot,L,li)
    if robot.mdh ~= L.mdh 
        if robot.mdh==0 printf("Robot model has standard DH parameters\n");
        else printf("Robot model has modified DH parameters\n");
        end
        if L.mdh==0 printf("New link has standard DH parameters\n");
        else printf("New link has modified DH parameters\n");
        end        
        error("Cannot replace because new link has different DH conventions from robot");
    end

    if type(li)~=1 error ("Link index must be a number."); end
    if  type(L)~= 17 then error("Wrong data type for link argument");
    end

    nlinks=robot.nj;
    if li<1 | li>nlinks
        emsg=sprintf("Valid link index is between 1 - %d",nlinks);
        error(emsg);
    else
         //robot.Link(li) = [];  // first delete the specified link
         robot.Link(li) = L;  // then replace with L
     end // if lidx<1 | lidx>nlinks+1
  
      robotcfg = '';
      for i=1:nlinks,   // automatic update for some variables
             if robot.Link(i).sigma then robotcfg=strcat([robotcfg,'P']);
             else robotcfg=strcat([robotcfg,'R']);
             end  
      end    
      robot.conf = robotcfg;   // update configuration string              
    
endfunction       

function ReplaceLinkHelp()
        printf("=============================================================\n");
        printf("Usage: robot=ReplaceLink(robot,L,li)\n\n");
        printf("replace link li of a robot with L\n");
        printf("\tEx: robot=ReplaceLink(robot,L,2); // replace link 2 \n");
        printf("=============================================================\n");
endfunction