//AppendLink.sci  append a new link to robot 
// www.controlsystemslab.com   August 2012
// usage: robot=AppendLink(robot,L,li);
// first create a new link using Link() function
// Example: 
// --> L = Link([0 0 1 0]);
// --> robot = AppendLink(robot,L,2) // add new link at position 2 from below
//                                      previous link 2 becomes link 3 etc.
// --> robot = AppendLink(robot,L)  // append link on top

function robot=AppendLink(robot,L,varargin)
    if argn(2)==0 then
        AppendLinkHelp();
        robot=[];
    else
        robot = _Append_Link(robot,L,varargin);
    end          
endfunction

function robot=appendlink(robot,L,varargin)
    if argn(2)==0 then
        AppendLinkHelp();
        robot=[];
    else
        robot = _Append_Link(robot,L,varargin);
    end 
endfunction
    
function robot=_Append_Link(robot,L,varargin)
    if robot.mdh ~= L.mdh 
        if robot.mdh==0 printf("Robot model has standard DH parameters\n");
        else printf("Robot model has modified DH parameters\n");
        end
        if L.mdh==0 printf("New link has standard DH parameters\n");
        else printf("New link has modified DH parameters\n");
        end        
        error("Cannot attach becuase new link has different DH conventions from robot");
    end
    if  type(L)~= 17 then error("Wrong data type for link argument");
    end

    varargin=varargin($);
    nlinks=size(robot.Link,1);  // number of links in this robot
    if length(varargin)==0  // no link index input, append at topmost
        robot.Link(nlinks+1) = L;
    else
        lidx = varargin(1);   // link index
        if type(lidx)~=1 error ("Link index must be a number.");
        end
        if lidx<1 | lidx>nlinks+1
            emsg=sprintf("The original robot has %d links. Valid link index is between 1 - %d",nlinks,nlinks+1);
            error(emsg);
        else
             for i=nlinks:-1:lidx  // push links above lidx upwards
                 robot.Link(i+1) = robot.Link(i);
             end
             robot.Link(lidx) = L;  // replace old link at lidx with new link
         end // if lidx<1 | lidx>nlinks+1
      end  // if length(varargin) == 0
      nlinks = nlinks+1;  // now robot has one more link than original
      robot.nj = nlinks;  // update number of links
      robotcfg = '';
      for i=1:nlinks,   // automatic update for some variables
             if robot.Link(i).sigma then robotcfg=strcat([robotcfg,'P']);
             else robotcfg=strcat([robotcfg,'R']);
             end  
      end    
      robot.conf = robotcfg;   // update configuration string              
    
endfunction       

function AppendLinkHelp()
        printf("=============================================================\n");
        printf("Usage: robot=AppendLink(robot,L,<link position>)\n\n");
        printf("First create a new link using Link() function, then append\n");
        printf("to robot at specified location \n");
        printf("\tEx: robot=AppendLink(robot,L,2); // inserts at link 2 \n");
        printf("\t(previous link 2 becomes link 3, etc.)\n");
        printf("\trobot=AppanedLink(robot,L); // inserts above topmost link\n");
        printf("=============================================================\n");
endfunction