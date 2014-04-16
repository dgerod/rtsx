//TRANIMATE.SCI Animate a coordinate frame
//
// TRANIMATE(P1, P2, OPTIONS) animates a 3D coordinate frame moving from pose P1
// to pose P2.  Poses P1 and P2 can be represented by:
//   - homogeneous transformation matrices (4x4)
//   - orthonormal rotation matrices (3x3)
//   - Quaternion
//
// TRANIMATE(P, OPTIONS) animates a coordinate frame moving from the identity pose
// to the pose P represented by any of the types listed above.
//
// TRANIMATE(PSEQ, OPTIONS) animates a trajectory, where PSEQ is any of
//   - homogeneous transformation matrix sequence (4x4xN)
//   - orthonormal rotation matrix sequence (3x3xN)
//   - Quaternion vector (Nx1)
//
// Options::

//  'nsteps', n  -- The number of steps along the path (default 50)
//  'grid' -- show grid
//  'world' -- show world coordinate 
//  'figure', n -- speciy window number
//   'speed', s -- speed number 1 (slowest) - 10 (fastest) 
//  'color','color_code' -- specify color
//  'linestyle', 'linestyle_code' -- specify line style

// See also TRPLOT.

// This function is inspired by The Robotics Toolbox for Matlab (RTB).
// (C) 1993-2011, by Peter I. Corke
// 

function tranimate(P2, varargin)

    //opt.fps = 10;

    framenum = 1;
    nsteps = 50;
    ShowGrid = 0;
    noreplay = 0;  // default to replay
    Hold = 0;
    setfignum = 0;  // flag if user set figure number
    PlotWorld = 0;
    HoldStart = 0;
    aspeed = 5;       // fastest
    fcolor = 5;    // red
    lstyle = 1;   // solid
    //opt.axis = [];
   Tx = [];
    fnum = max(winsid())+1;
    varnum=length(varargin);  // number of arguments
    for iv =1:varnum
        if type(varargin(1)~=10)
            Tx = varargin(1);  
        end    
        if varargin(iv)=='grid' then
            ShowGrid = 1;

        elseif varargin(iv)=='world' then
            PlotWorld = 1;
        elseif varargin(iv)=='hold' then
            Hold = 1;
            if ~setfignum then  // hold last window
                curwin = winsid();
                if curwin~=[] fnum = curwin(length(curwin));
                end
            end        
        elseif varargin(iv)=='noreplay' then
            noreplay = 1;
        elseif varargin(iv)=='holdstart' then
            HoldStart = 1;
        elseif varargin(iv)=='figure'& (iv<varnum) then  // can put figure number as arg
            fnum = varargin(iv+1); 
            setfignum = 1;
        elseif varargin(iv)=='frame'& (iv<varnum) then  // can put frame number as arg
            framenum = varargin(iv+1);             

        elseif varargin(iv)=='nsteps' & (iv<varnum) then
            nsteps = varargin(iv);
        elseif varargin(iv)=='speed' & (iv<varnum) then
            aspeed = varargin(iv+1);
            if aspeed < 1 
                aspeed = 1;
            elseif aspeed > 10
                aspeed = 10;
            end        
        elseif varargin(iv)=='color' & (iv<varnum) then
            fcolor = colormap(varargin(iv+1));
        
        elseif varargin(iv)=='linestyle' & (iv<varnum) then
            lstyle = linesmap(varargin(iv+1));   
        end  // if varargin(iv) == 'grid'
    end // for iv=1:varnum
    if find(winsid()==fnum)& ~Hold then           
        xdel(fnum);
    end   
   
    // [opt, args] = tb_optparse(opt, varargin);

    T1 = [];

    // convert quaternion and rotation matrix to hom transform
    if isquaternion(P2); 
        T2 = q2t(P2);   // convert quaternion to transform
        if ~isempty(Tx) & isquaternion(Tx)
            T1 = T2;
            Q2 = Tx;
            T2 = q2t(Q2); 
            //args = args(2:end);
        else
            T1 = eye(4,4);
        end
    elseif isrot(P2)
        T2 = r2t(P2);
        if ~isempty(Tx) & isrot(Tx)
            T1 = T2;
            T2 = r2t(Tx);
            //args = args(2:end);
        else
            T1 = eye(4,4);
        end
    elseif ishomog(P2)
        T2 = P2;
        if ~isempty(Tx) & ishomog(Tx)
            T1 = T2;
            T2 = Tx;
            //args = args(2:end);
        else
            T1 = eye(4,4);
        end
    end
    
    // at this point
    //   T1 is the initial pose
    //   T2 is the final pose
    //
    //  T2 may be a sequence
        
    if size(T2,3) > 1
        // tranimate(Ts)
        // we were passed a homog sequence
        //pause;
        //if ~isempty(T1)
        //    error('only 1 input argument if sequence specified');
        //end
        Ttraj = T2;
    else
        // tranimate(P1, P2)
        // create a path between them
        printf("\nGenerating trajectory (this process takes some time) ");
        Ttraj = ctraj(T1, T2, nsteps);
    end
    R = Ttraj(1:3,1:3,:);    // rotation sequences    
    d = Ttraj(1:3,4,:);     // origin sequences
   PlotScale = 0;
   nseqs = size(d,3);
   for i=1:nseqs
       PlotScale = max(PlotScale,norm(d(:,1,i)));
   end
   if PlotScale == 0 
       PlotScale = 1;
   end
    bmargin = 0.5*PlotScale;    // boundary margin
    xmin = min(d(1,1,:));    // calculate workspace
    xmax = max(d(1,1,:));
    ymin = min(d(2,1,:));
    ymax = max(d(2,1,:));
    zmin = min(d(3,1,:));
    zmax = max(d(3,1,:));
   
   printf('\nGenerating animation data ');
   // ------------- generate animation data -----------------
   for i = 1:nseqs
       [Xf, Yf, Zf]=GeoMakeFrame(d(:,:,i)',R(:,:,i),0.5*PlotScale);
       Fdata_X(:,:,i) = Xf;
       Fdata_Y(:,:,i) = Yf;
       Fdata_Z(:,:,i) = Zf;   
       printf(".");
        nl = modulo(i,80);
        if nl==0 then
            printf("\n");
        end       
   end
   // plot world coordinate (if specified)
   if PlotWorld
       wcolor = 2;
        //axisX = 0.1*PlotScale*[0 0 0;1 0 0];
        //axisY = 0.1*PlotScale*[0 0 0;0 1 0];
        //axisZ = 0.1*PlotScale*[0 0 0;0 0 1];
        [axisX,axisY,axisZ]=GeoMakeFrame([0 0 0],eye(3,3),0.4*PlotScale);
        figure(fnum);
        param3d1(axisX,axisY,axisZ); 
        hw=gce();
        
        axis_xw = hw.children(3)
        axis_yw = hw.children(2)
        axis_zw = hw.children(1)
        axis_xw.foreground = wcolor;
        axis_yw.foreground = wcolor;
        axis_zw.foreground = wcolor;
        axis_xw.polyline_style = 4;
        axis_yw.polyline_style = 4;
        axis_zw.polyline_style = 4;
        axis_xw.thickness = 1;
        axis_yw.thickness = 1;
        axis_zw.thickness = 1;

        // place axis label to figure

       xwtext = "Xw";
       ywtext = "Yw";
       zwtext = "Zw";
            

        xstring(0.5,0.5,xwtext);
        xpos = get("hdl");  // get the handle of the newly created object
        // Now set actual position
        xpos.data = [max(axisX),0,0];
        xpos.font_foreground = wcolor;        
        xstring(0.5,0.5,ywtext);
        ypos = get("hdl");
        ypos.data = [0,max(axisY),0];
        ypos.font_foreground = wcolor; 
        
        xstring(0.5,0.5,zwtext);
        zpos = get("hdl");
        zpos.data = [0,0,max(axisZ)];
        zpos.font_foreground = wcolor;         
    end      // if PlotWorld        
       
   if HoldStart
       hscolor = 6;
       // plot fixed first frame
        [axisX,axisY,axisZ]=GeoMakeFrame(d(:,:,1)',R(:,:,1),0.45*PlotScale);
        figure(fnum);
        param3d1(axisX,axisY,axisZ); 
        hs=gce();
        
        axis_xs = hs.children(3)
        axis_ys = hs.children(2)
        axis_zs = hs.children(1)
        axis_xs.foreground = hscolor;
        axis_ys.foreground = hscolor;
        axis_zs.foreground = hscolor;
        axis_xs.polyline_style = 4;
        axis_ys.polyline_style = 4;
        axis_zs.polyline_style = 4;
        axis_xs.thickness = 1;
        axis_ys.thickness = 1;
        axis_zs.thickness = 1;

        // place axis label to figure

       xstext = "X0";
       ystext = "Y0";
       zstext = "Z0";
            

        xstring(0.5,0.5,xstext);
        xspos = get("hdl");  // get the handle of the newly created object
        // Now set actual position
        xspos.data = [axisX(2,1),axisY(2,1),axisZ(2,1)];
        xspos.font_foreground = hscolor;        
        xstring(0.5,0.5,ystext);
        yspos = get("hdl");
        yspos.data = [axisX(2,2),axisY(2,2),axisZ(2,2)];;
        yspos.font_foreground = hscolor; 
        
        xstring(0.5,0.5,zstext);
        zspos = get("hdl");
        zspos.data = [axisX(2,3),axisY(2,3),axisZ(2,3)];
        zspos.font_foreground = hscolor;         
    end      // if HoldStart          

   // plot first frame for animation
   [axisX,axisY,axisZ]=GeoMakeFrame(d(:,:,1)',R(:,:,1),0.5*PlotScale);
   figure(fnum);
   param3d1(axisX,axisY,axisZ); 
   ht=gce();
   
   axis_xt = ht.children(3)
   axis_yt = ht.children(2)
   axis_zt = ht.children(1)
   axis_xt.foreground = fcolor;
   axis_yt.foreground = fcolor;
   axis_zt.foreground = fcolor;
   axis_xt.polyline_style = 4;
   axis_yt.polyline_style = 4;
   axis_zt.polyline_style = 4;
   axis_xt.line_style = lstyle;
   axis_yt.line_style = lstyle;
   axis_zt.line_style = lstyle;
   axis_xt.thickness = 1;
   axis_yt.thickness = 1;
   axis_zt.thickness = 1;

   // place axis label to figure

    if type(framenum)==1  // number
        xttext = sprintf("X%d", framenum);
        yttext = sprintf("Y%d", framenum);
        zttext = sprintf("Z%d", framenum);       
    elseif type(framenum)==10  // string
        xttext = sprintf("X%s", framenum);
        yttext = sprintf("Y%s", framenum);
        zttext = sprintf("Z%s", framenum);
    end    
   xstring(0.5,0.5,xttext);
   xtpos = get("hdl");  // get the handle of the newly created object
   // Now set actual position
   xtpos.data = [axisX(2,1),axisY(2,1),axisZ(2,1)];
   xtpos.font_foreground = fcolor;        
   xstring(0.5,0.5,yttext);
   ytpos = get("hdl");
   ytpos.data = [axisX(2,2),axisY(2,2),axisZ(2,2)];;
   ytpos.font_foreground = fcolor; 
   
   xstring(0.5,0.5,zttext);
   ztpos = get("hdl");
   ztpos.data = [axisX(2,3),axisY(2,3),axisZ(2,3)];
   ztpos.font_foreground = fcolor;         

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
   
     h_axes.rotation_angles = [70,45];

  
    h_axes.data_bounds = [xmin,ymin,zmin; xmax,ymax,zmax];

    if ShowGrid xgrid;
    end
// -------------- animation loop ---------------------
   replay = 1;
   dlyscale=speed2delay(aspeed);             
   while (replay)

          for i=1:nseqs
            drawlater();
             axis_xt.data = [Fdata_X(:,1,i), Fdata_Y(:,1,i), Fdata_Z(:,1,i)];
             axis_yt.data = [Fdata_X(:,2,i), Fdata_Y(:,2,i), Fdata_Z(:,2,i)];
             axis_zt.data = [Fdata_X(:,3,i), Fdata_Y(:,3,i), Fdata_Z(:,3,i)];
             xtpos.data = [Fdata_X(2,1,i),Fdata_Y(2,1,i),Fdata_Z(2,1,i)];
             ytpos.data = [Fdata_X(2,2,i),Fdata_Y(2,2,i),Fdata_Z(2,2,i)];    
             ztpos.data = [Fdata_X(2,3,i),Fdata_Y(2,3,i),Fdata_Z(2,3,i)];  
             drawnow();
             //dlyscale = round(1e8/exp(aspeed));
             for dly=1:dlyscale
              end                
          end
          if noreplay
              return;
          end    
              
           yrchoice = input("Enter [Y] to replay,[s] to change speed,  any other key to quit: ","s");
           if yrchoice=='s' | yrchoice =='S' then
               printf("\nCurrent speed is %d\n",aspeed);
               aspeed = input("Enter new speed from 1 - 10 : ");
               if aspeed < 1 
                   aspeed = 1;
               elseif aspeed > 10
                   aspeed = 10;
               end
               dlyscale=speed2delay(aspeed);
            elseif yrchoice=='y' | yrchoice=='Y' then
               replay = 1;  // replay
            else
                replay = 0;  // quit
            end
    end //while replay   
endfunction

function dlyscale = speed2delay(aspeed)
   select (aspeed)
   case 1
       dlyscale = 800000;
   case 2
       dlyscale = 500000;
   case 3
       dlyscale = 400000;
   case 4
       dlyscale = 300000;
   case 5
       dlyscale = 150000;
   case 6
       dlyscale = 80000;
   case 7
       dlyscale = 40000;
   case 8
       dlyscale = 20000;
   case 9
       dlyscale = 15000;
   case 10
       dlyscale = 10000;
   end                      
    
endfunction    







