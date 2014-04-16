//RemoveLink.sci  remove a robot link specified by li
// www.controlsystemslab.com   August 2012
// usage: robot=RemoveLink(robot,li);
// Example: 
// --> robot = RemoveLink(robot,2) // remove link 2

function robot=RemoveLink(robot,li)
    if argn(2)==0 then
       RemoveLinkHelp();
        robot=[];
    else
        robot = _Remove_Link(robot,li);
    end          
endfunction

function robot=removelink(robot,li)
    if argn(2)==0 then
       RemoveLinkHelp();
        robot=[];
    else
        robot = _Remove_Link(robot,li);
    end 
endfunction
    
function robot=_Remove_Link(robot,li)


    if type(li)~=1 error ("Link index must be a number.");
    end
    nlinks=robot.nj;
    if li<1 | li>nlinks
        emsg=sprintf("Valid link index is between 1 - %d",nlinks);
        error(emsg);
    else
         for i=li:nlinks-1  // shrink links above li downwards
             robot.Link(i) = robot.Link(i+1);
         end
         robot.Link(nlinks) = [];  // delete the excess
     end // if lidx<1 | lidx>nlinks+1
  
      nlinks = nlinks-1;  // now robot has one less link than original
      robot.nj = nlinks;  // update number of links
      robotcfg = '';
      for i=1:nlinks,   // automatic update for some variables
             if robot.Link(i).sigma then robotcfg=strcat([robotcfg,'P']);
             else robotcfg=strcat([robotcfg,'R']);
             end  
      end    
      robot.conf = robotcfg;   // update configuration string              
    
endfunction       

function RemoveLinkHelp()
        printf("=============================================================\n");
        printf("Usage: robot=RemoveLink(robot,li)\n\n");
        printf("remove link li of a robot\n");
        printf("\tEx: robot=RemoveLink(robot,2); // remove link 2 \n");
        printf("=============================================================\n");
endfunction