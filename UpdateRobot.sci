// UpdateRobot.sci   update robot data structure
// www.controlsystemslab.com  July 2012
// robot = UpdateRobot(robot,lindex,varargin) 
// Example:
// robot = UpdateRobot(robot,'link',1,'RP','R') // change link 1 to revolute

function robot = UpdateRobot(robot,varargin)    
    if argn(2)==0 then
        UpdateRobotHelp();
        robot=[];
    else
        
        robot = _Update_Robot(robot,varargin);
    end    
endfunction 


function robot = updaterobot(robot,varargin)    
    if argn(2)==0 then
        UpdateRobotHelp();
        robot=[];
    else
        
        robot = _Update_Robot(robot,varargin);
    end    
endfunction 

function robot = _Update_Robot(robot,varargin)

        varargin=varargin($);
        lidx = 1;  // link index, default = 1
        varnum=length(varargin);  // number of arguments
        
        if pmodulo(varnum,2) then  // number of arguments is odd. Error!
            error("Input argument number is odd. You must miss something!");
        else
            for iv =1:2:varnum-1  // select only command at odd position
                if type(varargin(iv))==10 then  // check if string (it should be!)
                    varargin(iv)=convstr(varargin(iv),'l'); // convert to lower 
                else
                    error("Parameter name must be a string (perhaps you forget to put it in quotes)!");                
                end
                select varargin(iv),
                    case 'name' then 
                        if  type(varargin(iv+1))==10 then
                             robot.name = varargin(iv+1);
                        else error("Robot name must be a string");
                        end
                    case 'manuf' then
                        if  type(varargin(iv+1))==10 then
                            robot.manuf = varargin(iv+1);
                        else error("Robot manufacturer must be a string");
                        end
                    case 'comment' then
                        if  type(varargin(iv+1))==10 then
                            robot.comment = varargin(iv+1);
                        else error("Robot comment must be a string");
                        end
                    case 'viewangle' then
                        if size(varargin(iv+1))==[1,2] then                       
                            robot.viewangle = varargin(iv+1);
                        else error("Value for robot viewangle must be 1 x 2 vector");
                        end
                    case 'base' then
                        if size(varargin(iv+1))== [4 4] then
                            robot.base = varargin(iv+1);
                        else error("Value for robot base must be 4 x 4 matrix");
                        end
                    case 'tool' then
                        if size(varargin(iv+1))== [4 4] then
                            robot.tool = varargin(iv+1);
                        else error("Value for robot tool must be 4 x 4 matrix");
                        end
                    case 'gravity' then
                        if size(varargin(iv+1))== [3 1] then
                            robot.gravity = varargin(iv+1);
                        else error("Value for robot gravity must be 3 x 1 vector");
                        end
                    case 'link' then
                        lidx = varargin(iv+1);
                        if lidx<1 | lidx>robot.nj  // check range
                            emsg = sprintf("Link index must be between 1 - %d",robot.nj);
                            error(emsg);
                        end
                    case 'rp' then 
                        jts = varargin(iv+1);
                        if jts=='R'|jts=='r'| jts==0
                                robot.Link(lidx).RP = 'R';
                                robot.Link(lidx).sigma = 0;
                         elseif jts=='P'|jts=='p'| jts==1
                                robot.Link(lidx).RP = 'P';
                                robot.Link(lidx).sigma = 1; 
                         else error("Joint type is either 0 (R) or 1 (P)");
                         end
                    case 'qlim' then
                        qlim = varargin(iv+1);
                        if size(qlim)==[1 2] 
                            robot.Link(lidx).qlim = qlim;
                        else error("Joint limits must be a 1x2 vector");
                        end
                    case 'linkparm' then
                        parm = varargin(iv+1);
                        if size(parm)==[1 4]|size(parm)==[1 5]
                            robot.Link(lidx).theta = parm(1);
                            robot.Link(lidx).d = parm(2);
                            robot.Link(lidx).a = parm(3);
                            robot.Link(lidx).alpha = parm(4);
                            if size(parm,2) == 5
                                robot.Link(lidx).offset = parm(5);  
                            end
                         else error("Link parameter must be a 1x4 or 1x5 vector");
                         end
                    case 'theta' then                         
                        parm = varargin(iv+1);
                        if type(parm)== 1 & size(parm)==[1 1] // a scalar
                            robot.Link(lidx).theta = parm;
                        else error("theta value must be a number");
                        end
                    case 'd' then 
                        parm = varargin(iv+1);
                        if type(parm)== 1 & size(parm)==[1 1] // a scalar
                            robot.Link(lidx).d = parm;
                        else error("d value must be a number");
                        end                            
                     case 'a' then 
                        parm = varargin(iv+1);
                        if type(parm)== 1 & size(parm)==[1 1] // a scalar
                            robot.Link(lidx).a = parm;
                        else error("a value must be a number");
                        end
                    case 'alpha' then 
                        parm = varargin(iv+1);
                        if type(parm)== 1 & size(parm)==[1 1] // a scalar
                            robot.Link(lidx).alpha = parm;
                        else error("alpha value must be a number");
                        end
                    case 'offset' then 
                        parm = varargin(iv+1);
                        if type(parm)== 1 & size(parm)==[1 1] // a scalar
                            robot.Link(lidx).offset = parm;
                        else error("offset value must be a number");
                        end
                    case 'dynparm' then
                        parm = varargin(iv+1);
                        if size(parm)==[1 15] 
                            robot.Link(lidx).r = parm(1,1:3);
                            
                            v = parm(1,4:9);
                            robot.Link(lidx).I = [v(1),v(4),v(6);v(4),v(2),v(5);v(6),v(5),v(3)];
                            robot.Link(lidx).m = parm(1,10);
                            robot.Link(lidx).Jm = parm(1,11);
                            robot.Link(lidx).G = parm(1,12);
                            robot.Link(lidx).B = parm(1,13);
                            robot.Link(lidx).Tc = parm(1,14:15);
                         else error("Dynamic parameters must be 1x15 vector");
                         end  
                    case 'r' then 
                        parm = varargin(iv+1);
                        if type(parm)== 1 & size(parm)==[1 3] // 1x3 vector
                            robot.Link(lidx).r = parm;
                        else error("r value must be a 1x3 vector");
                        end                                                                                                  case 'i' then 
                        parm = varargin(iv+1);
                        if type(parm)== 1 & size(parm)==[1 6] // 1x6 vector
                            v = parm;
                            robot.Link(lidx).I = [v(1),v(4),v(6);v(4),v(2),v(5);v(6),v(5),v(3)];
                        else error("I value must be a 1x6 vector");
                        end 
                    case 'm' then 
                        parm = varargin(iv+1);
                        if type(parm)== 1 & size(parm)==[1 1] // a scalar
                            robot.Link(lidx).m = parm;
                        else error("m value must be a number");
                        end                                                                                                 case 'jm' then 
                        parm = varargin(iv+1);
                        if type(parm)== 1 & size(parm)==[1 1] // a scalar
                            robot.Link(lidx).Jm = parm;
                        else error("Jm value must be a number");
                        end                                                                                                  case 'g' then 
                        parm = varargin(iv+1);
                        if type(parm)== 1 & size(parm)==[1 1] // a scalar
                            robot.Link(lidx).G = parm;
                        else error("G value must be a number");
                        end                                                                                                 case 'b' then 
                        parm = varargin(iv+1);
                        if type(parm)== 1 & size(parm)==[1 1] // a scalar
                            robot.Link(lidx).B = parm;
                        else error("B value must be a number");
                        end                                                                                                 case 'tc' then 
                        parm = varargin(iv+1);
                        if type(parm)== 1 & size(parm)==[1 2] // a 1x2 vector
                            robot.Link(lidx).Tc = parm;
                        else error("Tc value must be a 1x2 vector");
                        end                                                                                                                                              
                    else
                         
                        error("Invalid input parameter"); 
                end // select varargin(iv)
            end  // for iv=1:varnum
        end    // if pmodulo(varnum,2)   
        // update some robot parameters automatically
        nlinks = size(robot.Link,1);  // number of links
        robotcfg = '';
        for i=1:nlink,
             if robot.Link(i).sigma then robotcfg=strcat([robotcfg,'P']);
             else robotcfg=strcat([robotcfg,'R']);
             end  
        end    
        robot.conf = robotcfg;   // update configuration string      
endfunction

   

function UpdateRobotHelp()
        printf("=============================================================\n");    
        printf("Usage: robot=UpdateRobot(robot,<param1, value1>,<param2,value2> ...)\n");
        printf("where <parameter, value> pairs are (put all strings/characters in quotes)\n");
        printf("=====Set global robot parameters ====== \n");
        printf("\t<name, robot_name>: set robot name\n");
        printf("\t<manuf, robot_manuf>: set robot manufacturer\n");
        printf("\t<viewangle, [valpha,vtheta]>: set robot view angle (in degrees)\n");
        printf("\t<base,[Tb]>: set base frame (4x4 homog matrix)of robot\n");
        printf("\t<tool,[Tt]>: set tool frame (4x4 homog matrix)of robot\n");
        printf("\t<gravity,[rg]>: set robot gravity (3x1 vector)\n")
        printf("\n")
        printf("======Set individual link=======\n");
        printf("\t<link, i>: link selection \n");
        printf("\t<RP, R|P>: set link to revolute (R) or prismatic (P)\n");
        printf("\t<qlim,[min,max]>: set joint limit of selected link \n");
        printf("\t<linkparm,[theta d a alpha (offset)]>: set all link parameters of seleced link\n");
        printf("\t<dynparm,[r I m Jm G B Tc]>: set all dynamic link parameters of selected link\n");
        printf("\t<r|I|m|Jm|G|B|Tc,[value]>: set a dynamic link parameter of selected link\n");
        printf("=============================================================\n");
endfunction
