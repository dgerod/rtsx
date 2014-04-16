// robotinfo.sci  displays information of a robott 
// www.controlsystemslab.com   July 2012
// Usage: s=robotinfo(robot);
// where robot is a robot data structure created from SerialLink()

function robotinfo(robot)
    if (robot.name=='') then name="N/A";
    else name=robot.name;  
    end
    if (robot.manuf=='') then manuf="N/A";
    else manuf=robot.manuf;
    end
    printf("\n================= Robot Information =====================\n");
    printf("Robot name: %s\n",name); 
    printf("Manufacturer: %s\n",manuf);   
    printf("Number of joints: %d\n",robot.nj);
    printf("Configuration: %s\n",robot.conf);
    if (robot.mdh==0) then printf("Method: Standard DH\n");
    else print("Method: Modified DH\n");
    end
    // link parameters
    printf("+---+-----------+-----------+-----------+-----------+\n");
    printf("| j |   theta   |      d    |     a     |   alpha   |\n");
    printf("+---+-----------+-----------+-----------+-----------+\n");
    for(i=1:robot.nj)
        if  robot.Link(i).RP == 'R'
            printf("| %d |    q%d     |    %4.2f   |   %4.2f    |   %4.2f    |\n",i, ...
            i,robot.Link(i).d,robot.Link(i).a,robot.Link(i).alpha);
        else
            printf("| %d |    %4.2f   |    q%d     |   %4.2f    |   %4.2f    |\n",i, ...
            robot.Link(i).theta,i,robot.Link(i).a,robot.Link(i).alpha); 
        end           
    end
    printf("+---+-----------+-----------+-----------+-----------+\n");
    printf("Gravity = ");
    disp(robot.gravity);
    printf("Base = ");
    disp(robot.base);
    printf("Tool = ");
    disp(robot.tool);
    if robot.comment ~= '' 
        printf("Comment: %s\n",robot.comment);
    end
    
endfunction

function Robotinfo(robot)
    robotinfo(robot);
endfunction

function RobotInfo(robot)
    robotinfo(robot);
endfunction