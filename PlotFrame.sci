// PlotFrame.sci  plot a chain of coordinate frame
// www.controlsystemslab.com  August 2012


function plotframe(kc,varargin)
    if argn(2)==0
        PlotFrameHelp();
    else 
        _Plot_Frame(kc,varargin);
    end
endfunction

function PlotFrame(kc, varargin)
    if argn(2)==0
        PlotFrameHelp();
    else 
        _Plot_Frame(kc,varargin);
    end
endfunction

function _Plot_Frame(kc, varargin)
    if ~kc.completed    // check whether chain is completed
        error("Cannot plot an incomplete chain");
    end    
    varargin = varargin($);
    ShowGrid = 0;  // default plot to no grid
    ShowOrigin = 0; // default: no origin location display
    // retrieve variable arguments
    fnum = max(winsid())+1;
    varnum=length(varargin);  // number of arguments
    
    for iv =1:varnum
        if varargin(iv)=='grid' then
            ShowGrid = 1;
        end
        if varargin(iv)=='oinfo' then
            ShowOrigin = 1;
        end
        if varargin(iv)=='figure'& (iv<varnum) then  // can put figure number as arg
            fnum = varargin(iv+1); 
            if winsid()==fnum then           
                xdel(fnum);
            end
        end
    end    // for iv=1:varnum
    
    PlotScale = 0;    // global scale, for adjusting plot size
 
    bmargin = 0.1;     // margin for axis bound
    xmin = 0; xmax = 0; ymin = 0; ymax = 0; zmin = 0; zmax = 0; // plot bounds
    nf = size(kc.Frame,1);  // number of frames
    for i=1:nf  // extract data from chain
        Ti(:,:,i) = kc.Frame(i).Tabs;
        Ri(:,:,i) = Ti(1:3,1:3,i);    // rotation matrix
        di(:,i) = Ti(1:3,4,i);        // origin
        PlotScale=max(PlotScale,norm(di(:,i)));        
    end    
    //PlotScale=max(abs(di));
    if PlotScale==0 PlotScale = 1;  // cover when all frames have di=[0 0 0]' 
    end
    bmargin = 0.6*PlotScale;    // boundary margin
    xmin = min(di(1,:));    // calculate workspace
    xmax = max(di(1,:));
    ymin = min(di(2,:));
    ymax = max(di(2,:));
    zmin = min(di(3,:));
    zmax = max(di(3,:));
    ScaleVec = [0.3, 0.4, 0.5, 0.35, 0.45, 0.55];
        
    for i=1:nf    // create plot data for each frame
             is = modulo(i,6);
             scale = ScaleVec(is);

            [axisX,axisY,axisZ]=GeoMakeFrame(di(:,i)',Ri(:,:,i),scale*PlotScale);
            figure(fnum);
            param3d1(axisX,axisY,axisZ); 
            h=gce();
        
            axis_x = h.children(3)
            axis_y = h.children(2)
            axis_z = h.children(1)

            axis_x.foreground = i;
            axis_y.foreground = i;
            axis_z.foreground = i;
     
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
            xpos.font_foreground = i;
            // Now set actual position
            xpos.data = [axisX(2,1),axisY(2,1),axisZ(2,1)];

            xstring(0.5,0.5,yttext);
            ypos = get("hdl");
            ypos.font_foreground = i;            
            ypos.data = [axisX(2,2),axisY(2,2),axisZ(2,2)];

            xstring(0.5,0.5,zttext);
            zpos = get("hdl");
            zpos.font_foreground = i;            
            zpos.data = [axisX(2,3),axisY(2,3),axisZ(2,3)];
            // place frame name and location at origin
            if ShowOrigin 
                otext = sprintf("%s (%4.2f,%4.2f,%4.2f)",kc.Frame(i).name,di(1,i),di(2,i),di(3,i));
            else
                otext = sprintf("%s",kc.Frame(i).name);
            end
            xstring(0.5,0.5,otext);
            opos = get("hdl");
            opos.font_foreground = i;
            opos.data = [di(1,i), di(2,i), di(3,i)];

            if i<nf
                // draw line between frame
                Xl = [di(1,i);di(1,i+1)];                  
                Yl = [di(2,i); di(2,i+1)]; 
                Zl = [di(3,i); di(3,i+1)];                    
                figure(fnum);
                param3d(Xl,Yl,Zl);
                hl=gce();
    
                // hl.line_style = 8;
                hl.polyline_style = 4;
                hl.foreground = 8;
                hl.thickness = 2;
            end

      
    end    // for i=1:nf

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
    if kc.viewangle==[] then
         h_axes.rotation_angles = [63,26];
    else
        h_axes.rotation_angles = kc.viewangle;
    end

    h_axes.data_bounds = [xmin,ymin,zmin; xmax,ymax,zmax];
 
    if kc.name ~= '' then title(kc.name);
    end
    if ShowGrid then xgrid;
    end
    
endfunction    

function PlotFrameHelp()
        printf("=============================================================\n");    
        printf("Usage: PlotFrame(kc,<options>)\n\n");
        printf("\twhere kc is a completed kinematic chain to plot\n");
        printf("Available options: (put string in quotes)\n");
        printf("\tgrid: add grid to plot\n");
        printf("\t<figure, i>: plot on window number i\n");
        printf("\toinfo: show origin coordinates\n");
        printf("=============================================================\n");        
endfunction        