//AnimateRobot.sci  show a movie of robot manipulator in 3-D
// corresponding to a sequence of joint variables
// www.controlsystemslab.com   July 2012

function animaterobot(robot,q,varargin)
    if argn(2)==0 then
        AnimateRobotHelp();
       
    else
        
         _Animate_Robot(robot,q,varargin);
    end    

endfunction

function AnimateRobot(robot,q,varargin)
    if argn(2)==0 then
        AnimateRobotHelp();
       
    else
        
        _Animate_Robot(robot,q,varargin);
    end    
endfunction


function _Animate_Robot(robot,q, varargin)
    varargin=varargin($);
    ShowGrid = 0;  // default plot to no grid
    // retrieve variable arguments
    fnum = max(winsid())+1;
    varnum=length(varargin);  // number of arguments
    aspeed = 10;
    PlotWorld = 1;        // plot world and tool frame as default
    PlotTool = 1;
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
        elseif varargin(iv)=='speed' & (iv<varnum) then
            aspeed = varargin(iv+1);
            if aspeed < 1 
                aspeed = 1;
            elseif aspeed > 10
                aspeed = 10;
            end  
        elseif varargin(iv)=='figure'& (iv<varnum) then  // can put figure number as arg
            fnum = varargin(iv+1); 
            if winsid()==fnum then           
                xdel(fnum);
            end
        end
    end
    
    printf("Generating animation data");
    
    // general constants
    SideCount = 10;   // for cylinder shape
    PlotWorld = 1;     // flag to plot world coordinate
    PlotTool = 1;        // flag to plot tool coordinate
    PlotScale = 0.1;    // global scale, for adjusting plot size
    WorldBase = 0;    // flag whether world and base frame are the same 
    bmargin = 0.1;     // margin for axis bound
    MaxLength=max(max(robot.Link.d),max(robot.Link.a)); // the longest dimension
    // of all a's and d's variables
    xmin = 0; xmax = 0; ymin = 0; ymax = 0; zmin = 0; zmax = 0; // plot bounds
    

    L=robot.Link;
    nlinks=size(L,1);    // number of links 
    nseqs = size(q,1);    // number of sequences
    if nlinks~=size(q,2) then
       error("Numbers of links and joint variables do not match.")
    end
    [A_ji,T_ji]=Robot2hAT(robot,q);
    R_ji=hypermat([3 3 nseqs nlinks+2]);
    d_ji=hypermat([3 1 nseqs nlinks+2]);
    R_ji(:,:,:,:)=T_ji(1:3,1:3,:,:);
    d_ji(:,:,:,:)=T_ji(1:3,4,:,:);
    
    if A_ji(:,:,1,1)== eye(4,4) then  // base at world coordinate
        WorldBase = 1;
    end
    
    PlotScale=max(abs(d_ji));
    // plot world coordinate

    Radius = 0.05*MaxLength;    // for revolute joints
    Height = 0.13*MaxLength;
    JAxisLength = 1.4*Height;
    Lx = 0.1*MaxLength;        // for prismatic joints
    Ly = Lx;
    Lz = Lx;
    bmargin = 0.3*PlotScale;    // boundary margin
    xmin = min(d_ji(1,:,:,:));    // calculate workspace
    xmax = max(d_ji(1,:,:,:));
    ymin = min(d_ji(2,:,:,:));
    ymax = max(d_ji(2,:,:,:));
    zmin = min(d_ji(3,:,:,:));
    zmax = max(d_ji(3,:,:,:));

    if WorldBase==0 then  // world and base are different
        Ob=d_ji(:,:,1,1);  // origin of world frame
        xmin = min(xmin,Ob(1));
        xmax = max(xmax,Ob(1));
        ymin = min(ymin,Ob(2));
        ymax = max(ymax,Ob(2));
        zmin = min(zmin,Ob(3));
        zmax = max(zmax,Ob(3));
    end       
    
    
    // =============data containers for plotting =============
    RJdataS_X = hypermat([4 SideCount nseqs nlinks]);  // side of R joint
    RJdataS_Y = RJdataS_X;
    RJdataS_Z = RJdataS_X;
    RJdataB_X = hypermat([SideCount 2 nseqs nlinks]);  // bottom of R joint
    RJdataB_Y = RJdataB_X;
    RJdataB_Z = RJdataB_X;
    PJdata_X = hypermat([4 6 nseqs nlinks]); // P joint
    PJdata_Y = PJdata_X;
    PJdata_Z = PJdata_Y;
    Jaxdata_X = hypermat([2 1 nseqs nlinks]);  /// joint axis
    Jaxdata_Y = Jaxdata_X;
    Jaxdata_Z = Jaxdata_X;
    Ldata_X = hypermat([2 2 nseqs nlinks]);  // Link data
    Ldata_Y = Ldata_X;
    Ldata_Z = Ldata_X;
    Tdata_X = zeros(2,3,nseqs); // tool axis
    Tdata_Y = Tdata_X;
    Tdata_Z = Tdata_X;
    // ========================================================================
    for j=1:nseqs   // loop for each set of joint variable
        for i=1:nlinks    // create plot data for each link
            if robot.mdh == 0 then  // standard DH method
                if L(i).RP=='R' then  // revolute joint

                    VertexData = GeoVerMakeCylinder(d_ji(:,:,j,i)',R_ji(:,:,j,i),Radius,Height,SideCount);
                    [Xs,Ys,Zs,Xb,Yb,Zb] = GeoPatMakeCylinder(VertexData);
                    RJdataS_X(:,:,j,i)=Xs;
                    RJdataS_Y(:,:,j,i)=Ys;
                    RJdataS_Z(:,:,j,i)=Zs;
                    RJdataB_X(:,:,j,i)=Xb;
                    RJdataB_Y(:,:,j,i)=Yb;
                    RJdataB_Z(:,:,j,i)=Zb;
                    
                elseif L(i).RP == 'P' then // prismatic joint
                    //Lx = 0.2*L(i).d;    // represents joint by rectangular shape
                    //Ly = Lx;
                    //Lz = Lx;
                    //JAxisLength = 1.4*Lx;
                    bmargin = max(bmargin,Lx);
                    VertexData = GeoVerMakeBlock(d_ji(:,:,j,i)',R_ji(:,:,j,i),[Lx,Ly,Lz]);
                    [X,Y,Z] = GeoPatMakeBlock(VertexData);
                    PJdata_X(:,:,j,i)=X;
                    PJdata_Y(:,:,j,i)=Y;
                    PJdata_Z(:,:,j,i)=Z;
                else
                    errstr=sprintf("Invalid joint type at link %d",i);
                    error(errstr);
                end // if L(i).RP == 'R'
                if ~(i==1 & WorldBase==1) then          
                    // compute joint axis
  
                    Jaxdata = GeoMakeJointAxis(d_ji(:,:,j,i)',R_ji(:,:,j,i),JAxisLength);
                    Jaxdata_X(:,:,j,i)=Jaxdata(:,1);
                    Jaxdata_Y(:,:,j,i)=Jaxdata(:,2);
                    Jaxdata_Z(:,:,j,i)=Jaxdata(:,3);

                end 

                // compute  link data
                
                pxz = Xlocate4(robot,q,A_ji,T_ji,j,i)    // find intersection of x_i and z_i-1
                Ldata_X(:,:,j,i) = [d_ji(1,:,j,i) d_ji(1,:,j,i+1);
                      pxz(1,1) pxz(1,1)];
                Ldata_Y(:,:,j,i) = [d_ji(2,:,j,i) d_ji(2,:,j,i+1);
                      pxz(2,1) pxz(2,1)];
                Ldata_Z(:,:,j,i) = [d_ji(3,:,j,i) d_ji(3,:,j,i+1);
                      pxz(3,1) pxz(3,1)];
            else
               error("The current version of RTSX supports only standard DH configuration\n")
           end   // if robot.mdh==0      
        end  // for i=i:nlinks
     
        // compute tool coordinate
        if PlotTool then

            [Xt,Yt,Zt]=GeoMakeFrame(d_ji(:,:,j,nlinks+2)',R_ji(:,:,j,nlinks+2),0.2*PlotScale);
            Tdata_X(:,:,j)=Xt;
            Tdata_Y(:,:,j)=Yt;
            Tdata_Z(:,:,j)=Zt;
        end
    printf(".");
    nl = modulo(j,80);
    if nl==0 then
        printf("\n");
    end
     end  // for j=1:nseqs
    printf("\n");
     // ================ draw initial figure ===============

     // -----------------Draw side patches -------------------
     for (i=1:nlinks)
         if  L(i).RP == 'R' then
             // draw side patches
             figure(fnum);
             plot3d(RJdataS_X(:,:,1,i),RJdataS_Y(:,:,1,i),RJdataS_Z(:,:,1,i));
             hr_s(i) = gce();
             hr_s(i).color_mode = 4;
             hr_s(i).foreground = 1;
             hr_s(i).hiddencolor = 5;
             // draw bottom patches
             figure(fnum);
             plot3d(RJdataB_X(:,:,1,i),RJdataB_Y(:,:,1,i),RJdataB_Z(:,:,1,i));
             hr_b(i) = gce();
             hr_b(i).color_mode = 4;
             hr_b(i).foreground = 1;
             hr_b(i).hiddencolor = 5;
        else // prismatic joint
             figure(fnum);
             plot3d(PJdata_X(:,:,1,i),PJdata_Y(:,:,1,i),PJdata_Z(:,:,1,i));
             hp(i) = gce();
             hp(i).color_mode = 4;
             hp(i).foreground = 1;
             hp(i).hiddencolor = 4;
        end 
        // --------------- plot joint axis ------------------------
        if ~(i==1 & WorldBase==1) then
            
            figure(fnum);
        
            param3d(Jaxdata_X(:,:,1,i),Jaxdata_Y(:,:,1,i),Jaxdata_Z(:,:,1,i));
            h_jax(i) = gce();
            jaxis(i) = h_jax(i).children;
            jaxis(i).foreground = 1;
            jaxis(i).polyline_style = 4;
            jaxis(i).thickness = 1;
            // label for joint axis
            jaxtext=sprintf("Z%d",i-1);
            xstring(0.5,0.5,jaxtext);
            jzpos(i) = get("hdl");  // get the handle of the newly created object
            // Now set actual position
            jzpos(i).data = [Jaxdata_X(2,:,1,i),Jaxdata_Y(2,:,1,i),Jaxdata_Z(2,:,1,i)];
         end   
 
         // ----------------- plot link ---------------------- 
         figure(fnum);
         param3d1(Ldata_X(:,:,1,i),Ldata_Y(:,:,1,i),Ldata_Z(:,:,1,i));
         hl(i)=gce();
         line_ai(i)=hl(i).children(1);
         line_di(i)=hl(i).children(2);
         line_ai(i).foreground = 2;
         if L(i).RP=='R' then
             line_di(i).foreground = 2;  // blue link for (R)
         else
             line_di(i).foreground = 7;  // yellow link for (P)
         end
         line_ai(i).thickness = 3;
         line_di(i).thickness = 3;
     end // of for(i=1:nlinks) loop
     if PlotTool then
            figure(fnum);
            param3d1(Tdata_X(:,:,1),Tdata_Y(:,:,1),Tdata_Z(:,:,1)); 
            ht=gce();
        
            axis_xt = ht.children(3)
            axis_yt = ht.children(2)
            axis_zt = ht.children(1)
            axis_xt.foreground = 4;
            axis_yt.foreground = 4;
            axis_zt.foreground = 4;
            axis_xt.polyline_style = 4;
            axis_yt.polyline_style = 4;
            axis_zt.polyline_style = 4;
            axis_xt.thickness = 1;
            axis_yt.thickness = 1;
            axis_zt.thickness = 1;

            // place axis label to figure
            xttext = "Xt";
            yttext = "Yt";
            zttext = "Zt";

            xstring(0.5,0.5,xttext);
            xtpos = get("hdl");  // get the handle of the newly created object
            // Now set actual position
            xtpos.data = [Tdata_X(2,1,1),Tdata_Y(2,1,1),Tdata_Z(2,1,1)];

            xstring(0.5,0.5,yttext);
            ytpos = get("hdl");
            ytpos.data = [Tdata_X(2,2,1),Tdata_Y(2,2,1),Tdata_Z(2,2,1)];

            xstring(0.5,0.5,zttext);
            ztpos = get("hdl");
            ztpos.data = [Tdata_X(2,3,1),Tdata_Y(2,3,1),Tdata_Z(2,3,1)];
     end  // if PlotTool

     // ------------- plot world coordinate ---------------
     if PlotWorld then
        //axisX = 0.1*PlotScale*[0 0 0;1 0 0];
        //axisY = 0.1*PlotScale*[0 0 0;0 1 0];
        //axisZ = 0.1*PlotScale*[0 0 0;0 0 1];
        [axisX,axisY,axisZ]=GeoMakeFrame([0 0 0],eye(3,3),0.2*PlotScale);
        figure(fnum);
        param3d1(axisX,axisY,axisZ); //,35,45,"@x@y@z",[1,4]);
        hw=gce();
        
        axis_xw = hw.children(3)
        axis_yw = hw.children(2)
        axis_zw = hw.children(1)
        axis_xw.foreground = 3;
        axis_yw.foreground = 3;
        axis_zw.foreground = 3;
        axis_xw.polyline_style = 4;
        axis_yw.polyline_style = 4;
        axis_zw.polyline_style = 4;
        axis_xw.thickness = 2;
        axis_yw.thickness = 2;
        axis_zw.thickness = 2;

        // place axis label to figure
        if WorldBase then // base and world frame are the same
            xwtext = "X0";
            ywtext = "Y0";
            zwtext = "Z0";
            
        else
            xwtext = "Xw";
            ywtext = "Yw";
            zwtext = "Zw";
            
        end
        xstring(0.5,0.5,xwtext);
        xpos = get("hdl");  // get the handle of the newly created object
        // Now set actual position
        xpos.data = [max(axisX),0,0];
        
        xstring(0.5,0.5,ywtext);
        ypos = get("hdl");
        ypos.data = [0,max(axisY),0];

        xstring(0.5,0.5,zwtext);
        zpos = get("hdl");
        zpos.data = [0,0,max(axisZ)];
    end      // if PlotWorld 
    
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
    if ShowGrid xgrid;
    end 
    //halt('Press any key to animate ...')
    //yrchoice=input("Enter anything to start, [Q] to quit: ","s");
    //if (yrchoice=='q'| yrchoice=='Q') then
    //     replay = 0;  // quit
     //else
     //    replay = 1;
     //end
    replay = 1;
    dlyscale=speed2delay(aspeed);       
    while (replay)
   
        // Animation Loop
        for j=1:nseqs

            drawlater();
            for i=1:nlinks
                if L(i).RP=='R' then  //revolute joint
                    hr_s(i).data.x = RJdataS_X(:,:,j,i); // side patches
                    hr_s(i).data.y = RJdataS_Y(:,:,j,i); 
                    hr_s(i).data.z = RJdataS_Z(:,:,j,i);
                    hr_b(i).data.x = RJdataB_X(:,:,j,i); // bottom patches
                    hr_b(i).data.y = RJdataB_Y(:,:,j,i);
                    hr_b(i).data.z = RJdataB_Z(:,:,j,i);
                else  // prismatic joint
                    hp(i).data.x = PJdata_X(:,:,j,i);    // block
                    hp(i).data.y = PJdata_Y(:,:,j,i);
                    hp(i).data.z = PJdata_Z(:,:,j,i);
                end
                // joint axes
                if ~(i==1 & WorldBase==1) then
 
                    h_jax(i).data = [Jaxdata_X(:,:,j,i) Jaxdata_Y(:,:,j,i) Jaxdata_Z(:,:,j,i)]; 
                    jzpos(i).data = [Jaxdata_X(2,:,j,i),Jaxdata_Y(2,:,j,i),Jaxdata_Z(2,:,j,i)];
                end
                // links
                line_ai(i).data = [Ldata_X(:,2,j,i) Ldata_Y(:,2,j,i) Ldata_Z(:,2,j,i)];
                line_di(i).data = [Ldata_X(:,1,j,i) Ldata_Y(:,1,j,i) Ldata_Z(:,1,j,i)];
 
                // tool frame
                axis_xt.data = [Tdata_X(:,1,j) Tdata_Y(:,1,j) Tdata_Z(:,1,j)];
                axis_yt.data = [Tdata_X(:,2,j) Tdata_Y(:,2,j) Tdata_Z(:,2,j)];
                axis_zt.data = [Tdata_X(:,3,j) Tdata_Y(:,3,j) Tdata_Z(:,3,j)];

                xtpos.data = [Tdata_X(2,1,j),Tdata_Y(2,1,j),Tdata_Z(2,1,j)];
                ytpos.data = [Tdata_X(2,2,j),Tdata_Y(2,2,j),Tdata_Z(2,2,j)];
                ztpos.data = [Tdata_X(2,3,j),Tdata_Y(2,3,j),Tdata_Z(2,3,j)];
            
            end    // for i=1:nlinks
    
            drawnow();
            for dly=1:dlyscale
            end   
        end // for j=1:nseqs
           yrchoice = input("Enter [Y] to replay,[s] to change speed,  any other key to quit: ","s");
           if yrchoice=='s' | yrchoice =='S' then
               newspeed = input("Enter new speed from 1 - 10 : ");
               if newspeed < 1 
                   newspeed = 1;
               elseif newspeed > 10
                   newspeed = 10;
               end
               dlyscale=speed2delay(newspeed);
            elseif yrchoice=='y' | yrchoice=='Y' then
               replay = 1;  // replay
            else
                replay = 0;  // quit
            end
    end //while replay

    //adata=[];  // added later
endfunction 

function AnimateRobotHelp()
        printf("=============================================================\n");
        printf("Usage: AnimateRobot(robot,qs,<options>)\n\n");
        printf("\twhere robot is the robot model to animate with joint variable sequence qs\n");
        printf("\tqs must be a ns x nj vector, where ns = number of setpoints and nj = number of joints of robot\n");
        printf("Available options: (put string in quotes)\n");
        printf("\tgrid: add grid to plot\n");
        printf("\t<figure, i>: plot on window number i\n");
        printf("=============================================================\n");
endfunction





