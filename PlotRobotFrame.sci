//PlotRobotFrame.sci  sketch a robot frame structure in 3-D
// www.controlsystemslab.com   July 2012

function [T]=plotrobotframe(robot,q,varargin)
    if argn(2)==0 then
        PlotRobotFrameHelp();
        T=[];
    else
        
        T = _Plot_Robot_Frame(robot,q,varargin);
    end    

endfunction

function [T]=PlotRobotFrame(robot,q,varargin)
    if argn(2)==0 then
        PlotRobotFrameHelp();
        T=[];
    else
        
        T = _Plot_Robot_Frame(robot,q,varargin);
    end    
endfunction



function [T]=_Plot_Robot_Frame(robot,q, varargin)
    varargin=varargin($);
    ShowGrid = 0;  // default plot to no grid
    // retrieve variable arguments
    Hold = 0;     // whether to hold previous plot
    fnum = max(winsid())+1;
    setfignum=0;
    varnum=length(varargin);  // number of arguments    
    PlotWorld = 1;        // plot world and tool frame as default
    if robot.tool==eye(4,4) then
        PlotTool = 0;
    else
        PlotTool = 1;    // flag to plot tool coordinate
    end
    for iv =1:varnum
        if type(varargin(iv))==10 then  // string
            varargin(iv)=convstr(varargin(iv),'l');   // convert to lower case
        end
        if varargin(iv)=='grid' then
            ShowGrid = 1;       
        elseif varargin(iv)== 'noworld' then
            PlotWorld = 0;
        elseif varargin(iv)== 'notool' then
            PlotTool = 0;
        elseif varargin(iv)=='hold' then
            Hold = 1;   
            if ~setfignum then  // hold last window
                curwin = winsid();
                if curwin~=[] fnum = curwin(length(curwin));
                end
            end                      
        elseif varargin(iv)=='figure'& (iv<varnum) then  // can put figure number as arg
            fnum = varargin(iv+1); 
            setfignum = 1;
        end
    end
    if find(winsid()==fnum) & ~Hold then         
        xdel(fnum);
    end
    // general constants
    //SideCount = 10;   // for cylinder shape
    //PlotWorld = 1;     // flag to plot world coordinate


    PlotScale = 0;    // global scale, for adjusting plot size
    WorldBase = 0;    // flag whether world and base frame are the same 
    bmargin = 0.1;     // margin for axis bound
    MaxLength=max(max(robot.Link.d),max(robot.Link.a)); // the longest dimension
    // of all a's and d's
    // variables
    xmin = 0; xmax = 0; ymin = 0; ymax = 0; zmin = 0; zmax = 0; // plot bounds
    
    // extract data from robot
    Tb = robot.base;
    Rb = Tb(1:3,1:3);  // base orientation
    Ob = Tb(1:3,4);    // base position
    Tt = robot.tool;
    Rt = Tt(1:3,1:3);
    L=robot.Link;
    nlinks=size(L,1);    // number of links 
    if nlinks~=size(q,2) then
          error("Numbers of links and joint variables do not match.")
    end 
    A=Link2AT(L,q);
    Ti = zeros(4,4,nlinks+2);
    Ri = zeros(3,3,nlinks+2);
    di = zeros(3,1,nlinks+2);
    Ti(:,:,1)=Tb;
    Ri(:,:,1)= Rb;
    di(:,1)=Ob;
    PlotScale=max(PlotScale,norm(di(:,1)));    
    //Ti(:,:,1)=A(:,:,1);
    //Ri(:,:,1)=Ti(1:3,1:3,1);    // orientation of joint 1
    //di(:,1)=Ti(1:3,4,1);        // location of joint 1
    for i=2:nlinks+1
        Ti(:,:,i)= Ti(:,:,i-1)*A(:,:,i-1);
        Ri(:,:,i)=Ti(1:3,1:3,i);    // orientation of joint 1
        di(:,i)=Ti(1:3,4,i);        // location of joint 1  
        PlotScale=max(PlotScale,norm(di(:,i)));             
    end
    Ti(:,:,nlinks+2)=Ti(:,:,nlinks+1)*Tt;
    Ri(:,:,nlinks+2)=Ti(1:3,1:3,nlinks+2);
    di(:,nlinks+2)=Ti(1:3,4,nlinks+2);
    PlotScale=max(PlotScale,norm(di(:,nlinks+2)));    
    //a=get("current_axes");    
    figure(fnum);
    //PlotScale=max(abs(di));
     ScaleVec = [0.3, 0.4, 0.5, 0.35, 0.45, 0.55, 0.3, 0.4, 0.5, 0.35, 0.45, 0.55];
    // plot world coordinate
    if PlotWorld then
        //axisX = 0.1*PlotScale*[0 0 0;1 0 0];
        //axisY = 0.1*PlotScale*[0 0 0;0 1 0];
        //axisZ = 0.1*PlotScale*[0 0 0;0 0 1];
        [axisX,axisY,axisZ]=GeoMakeFrame([0 0 0],eye(3,3),0.45*PlotScale);
        figure(fnum);
        param3d1(axisX,axisY,axisZ); //,35,45,"@x@y@z",[1,4]);
        h=gce();
        
        axis_x = h.children(3)
        axis_y = h.children(2)
        axis_z = h.children(1)
        axis_x.foreground = 1;
        axis_y.foreground = 1;
        axis_z.foreground = 1;
        axis_x.polyline_style = 4;
        axis_y.polyline_style = 4;
        axis_z.polyline_style = 4;
        axis_x.thickness = 2;
        axis_y.thickness = 2;
        axis_z.thickness = 2;

        // place axis label to figure
        if Tb == eye(4,4) then // base and world frame are the same
            xwtext = "X0";
            ywtext = "Y0";
            zwtext = "Z0";
            WorldBase = 1;
        else
            xwtext = "Xw";
            ywtext = "Yw";
            zwtext = "Zw";
            WorldBase = 0;
        end
        xstring(0.5,0.5,xwtext);
        xpos = get("hdl");  // get the handle of the newly created object
        xpos.font_foreground = 1;
        // Now set actual position
        xpos.data = [max(axisX),0,0];
        
        xstring(0.5,0.5,ywtext);
        ypos = get("hdl");
        ypos.font_foreground = 1;
        ypos.data = [0,max(axisY),0];

        xstring(0.5,0.5,zwtext);
        zpos = get("hdl");
        zpos.font_foreground = 1;
        zpos.data = [0,0,max(axisZ)];
    end
    //Radius = 0.05*MaxLength;    // for revolute joints
    //Height = 0.1*MaxLength;
    //JAxisLength = 1.4*Height;
    //Lx = 0.05*MaxLength;        // for prismatic joints
    //Ly = Lx;
    //Lz = Lx;
    bmargin = 0.6*PlotScale;    // boundary margin
    xmin = min(di(1,:));    // calculate workspace
    xmax = max(di(1,:));
    ymin = min(di(2,:));
    ymax = max(di(2,:));
    zmin = min(di(3,:));
    zmax = max(di(3,:));
    if WorldBase==0 then  // world and base are different
        xmin = min(xmin,Ob(1));
        xmax = max(xmax,Ob(1));
        ymin = min(ymin,Ob(2));
        ymax = max(ymax,Ob(2));
        zmin = min(zmin,Ob(3));
        zmax = max(zmax,Ob(3));
    end   
    
    for i=1:nlinks    // create plot data for each link
 
        if robot.mdh == 0 then  // standard DH method
            
            [axisX,axisY,axisZ]=GeoMakeFrame(di(:,i+1)',Ri(:,:,i+1),ScaleVec(i)*PlotScale);
            figure(fnum);
            param3d1(axisX,axisY,axisZ); 
            h=gce();
        
            axis_x = h.children(3)
            axis_y = h.children(2)
            axis_z = h.children(1)
            if i~= 3 then        // reserve red color to primatic link
                axis_x.foreground = i+2;
                axis_y.foreground = i+2;
                axis_z.foreground = i+2;
                font_color = i+2;
            else
                axis_x.foreground = 2;
                axis_y.foreground = 2;
                axis_z.foreground = 2;
                font_color = 2;
            end                
            axis_x.polyline_style = 4;
            axis_y.polyline_style = 4;
            axis_z.polyline_style = 4;
            axis_x.thickness = 2;
            axis_y.thickness = 2;
            axis_z.thickness = 2;

            // place axis label to figure
            xttext = sprintf("X%d",i);
            yttext = sprintf("Y%d",i);
            zttext = sprintf("Z%d",i);

            xstring(0.5,0.5,xttext);
            xpos = get("hdl");  // get the handle of the newly created object
            xpos.font_foreground = font_color;
            // Now set actual position
            xpos.data = [axisX(2,1),axisY(2,1),axisZ(2,1)];

            xstring(0.5,0.5,yttext);
            ypos = get("hdl");
            ypos.font_foreground = font_color;
            ypos.data = [axisX(2,2),axisY(2,2),axisZ(2,2)];

            xstring(0.5,0.5,zttext);
            zpos = get("hdl");
            zpos.font_foreground = font_color;
            zpos.data = [axisX(2,3),axisY(2,3),axisZ(2,3)];

            // draw links
            pxz = Xlocate(robot,q, A,Ti,i)    // find intersection of x_i and z_i-1
            Xl = [di(1,i) di(1,i+1);
                  pxz(1,1) pxz(1,1)];
            Yl = [di(2,i) di(2,i+1);
                  pxz(2,1) pxz(2,1)];
            Zl = [di(3,i) di(3,i+1);
                  pxz(3,1) pxz(3,1)];
                    
            figure(fnum);
            param3d1(Xl,Yl,Zl);
            hl=gce();
            line_ai=hl.children(1);
            line_di=hl.children(2);
            line_ai.line_style = 8;
            line_di.line_style = 8;
            line_ai.foreground = 2;
            line_di.foreground = 2;
            line_ai.thickness = 2;
            line_di.thickness = 2;
            if L(i).RP=='P' then  
                //line_di.line_style = 7;
                //line_di.thickness = 1;
                line_di.foreground = 7;  // yellow
                dlabel = sprintf("d%d",i);
                 xstring(0.5,0.5,dlabel);
                 dlpos = get("hdl");  // get the handle of the newly created object
                // Now set actual position
                dc = linecenter(pxz,di(:,i));
                 dlpos.data = dc';               
            end
       else
           error("The current version of RTSX supports only standard DH configuration\n")
       end         
    end
    // plot tool coordinate
    if PlotTool then
        toolcolor = nlinks+3;
        [axisX,axisY,axisZ]=GeoMakeFrame(di(:,nlinks+2)',Ri(:,:,nlinks+2),ScaleVec(nlinks+1)*PlotScale);
        figure(fnum);
        param3d1(axisX,axisY,axisZ); 
        h=gce();
        
        axis_x = h.children(3)
        axis_y = h.children(2)
        axis_z = h.children(1)
        axis_x.foreground = toolcolor;
        axis_y.foreground = toolcolor;
        axis_z.foreground = toolcolor;
        axis_x.polyline_style = 4;
        axis_y.polyline_style = 4;
        axis_z.polyline_style = 4;
        axis_x.thickness = 2;
        axis_y.thickness = 2;
        axis_z.thickness = 2;

        // place axis label to figure
        xttext = "Xt";
        yttext = "Yt";
        zttext = "Zt";

        xstring(0.5,0.5,xttext);
        xpos = get("hdl");  // get the handle of the newly created object
        xpos.font_foreground = toolcolor;
        // Now set actual position
        xpos.data = [axisX(2,1),axisY(2,1),axisZ(2,1)];

        xstring(0.5,0.5,yttext);
        ypos = get("hdl");
        ypos.font_foreground = toolcolor;
        ypos.data = [axisX(2,2),axisY(2,2),axisZ(2,2)];

        xstring(0.5,0.5,zttext);
        zpos = get("hdl");
        zpos.font_foreground = toolcolor;
        zpos.data = [axisX(2,3),axisY(2,3),axisZ(2,3)];
    end    

    // adjust margin to bounds



    xmin = xmin - bmargin;
    xmax = xmax + bmargin;
    ymin = ymin - bmargin;
    ymax = ymax + bmargin;
    zmin = zmin - bmargin;
    zmax = zmax + bmargin;


    // Axes settings
    xlabel("x",'fontsize',2);
    ylabel("y",'fontsize',2);
    zlabel("z",'fontsize',2);
    h_axes = gca();
    h_axes.font_size = 2;
    h_axes.isoview = "on";
    h_axes.box = "off";
    if robot.viewangle==[] then
         h_axes.rotation_angles = [63.5,-127];
    else
        h_axes.rotation_angles = robot.viewangle;
    end

    h_axes.data_bounds = [xmin,ymin,zmin; xmax,ymax,zmax];
 
    if robot.name ~= '' then title(robot.name);
    end
    if ShowGrid then xgrid;
    end

    T=Ti(:,:,nlinks+2);    // return homogeneous matrix from base to tool
    T=clean(T);

endfunction 

function PlotRobotFrameHelp()
        printf("=============================================================\n");    
        printf("Usage: [T]=PlotRobotFrame(robot,q,<options>)\n\n");
        printf("\twhere robot is the robot model to plot with joint variable q\n");
        printf("\tq must be a 1 x nj vector, where nj = number of joints of robot\n");
        printf("Available options: (put string in quotes)\n");
        printf("\tgrid: add grid to plot\n");
        printf("\t<figure, i>: plot on window number i\n");
        printf("=============================================================\n");        
endfunction
