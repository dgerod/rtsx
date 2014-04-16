// PlotResolveFrame.sci  plot 2 closed-loop chains of coordinate frames and 
// solve for missing link
// www.controlsystemslab.com  August 2012


function [T_rel,T_abs,kc1]=plotresolveframe(kc1,kc2, varargin)
    if argn(2)==0
        PlotResolveFrameHelp();
    else 
        [T_rel,T_abs,kc1]=_Plot_Resolve_Frame(kc1,kc2, varargin);
    end
endfunction

function [T_rel,T_abs,kc1]=PlotResolveFrame(kc1,kc2, varargin)
    if argn(2)==0
        PlotResolveFrameHelp();
    else 
        [T_rel,T_abs,kc1]=_Plot_Resolve_Frame(kc1,kc2, varargin);
    end
endfunction

function [T_rel,T_abs,kc1]=_Plot_Resolve_Frame(kc1,kc2, varargin)
    

    varargin = varargin($);
    ShowGrid = 0;  // default plot to no grid
    ShowOrigin = 0; // default: no origin location display
    missingframe = 0;  // keep missing frame number of kc1
    font_color = 1;   // axis label color
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
    
    if  ishomog(kc2,'valid') then  // kc2 is just a 4 x 4 homogeneous matrix
        F(1)=Frame(eye(4,4));  // create new chain for kc2
        F(2)=Frame(kc2);
        kc2=SerialFrame(F,'name','Chain 2');
    end

    if ~kc2.completed    // check whether chain 2 is completed
        error("2nd argument must be a complete chain");
    end    
    nf1 = size(kc1.Frame,1);  // number of frames in chain 1
    nf2 = size(kc2.Frame,1); // number of frames in chain 2
    if ~kc1.completed
        missingframe = kc1.missingframe;
        [T_rel,T_abs,kc1]=ResolveFrame(kc1,kc2);
  
    end
    
    if ~isequal(kc1.Frame(1).Tabs,kc2.Frame(1).Tabs)   
        error("Both chain must have same base frame");
    end
    if ~isequal(kc1.Frame(nf1).Tabs,kc2.Frame(nf2).Tabs)   
        error("Both chain must have same end frame");
    end     
    PlotScale = 0;    // global scale, for adjusting plot size
 
    bmargin = 0.1;     // margin for axis bound
    xmin = 0; xmax = 0; ymin = 0; ymax = 0; zmin = 0; zmax = 0; // plot bounds

    
    for i=1:nf1  // extract data from chain1
        T1(:,:,i) = kc1.Frame(i).Tabs;
        R1(:,:,i) = T1(1:3,1:3,i);    // rotation matrix
        d1(:,i) = T1(1:3,4,i);        // origin
        PlotScale=max(PlotScale,norm(d1(:,i)));
    end    
    for i=1:nf2  // extract data from chain1
        T2(:,:,i) = kc2.Frame(i).Tabs;
        R2(:,:,i) = T2(1:3,1:3,i);    // rotation matrix
        d2(:,i) = T2(1:3,4,i);        // origin
        PlotScale=max(PlotScale,norm(d2(:,i)));        
    end  
    //PlotScale=max(abs([d1 d2]));
    if PlotScale==0 PlotScale = 1;  // cover when all frames have di=[0 0 0]' 
    end
    bmargin = 0.6*PlotScale;    // boundary margin
    xmin = min([d1(1,:) d2(1,:)]);    // calculate plot space
    xmax = max([d1(1,:) d2(1,:)]);
    ymin = min([d1(2,:) d2(2,:)]);
    ymax = max([d1(2,:) d2(2,:)]);
    zmin = min([d1(3,:) d2(3,:)]);
    zmax = max([d1(3,:) d2(3,:)]);
    //ScaleVec = [0.3, 0.4, 0.5, 0.35, 0.45, 0.55];
    ScaleVec = [0.2, 0.3, 0.4,0.25, 0.35, 0.45];    
    // =============== Plot chain 1 =======================
    // use red color for chain 1 except base = back, end =magenta
    // missing link = dotted green
    for i=1:nf1    // create plot data for each frame in chain 1
             is = modulo(i,6);
             scale = ScaleVec(is);

            [axisX,axisY,axisZ]=GeoMakeFrame(d1(:,i)',R1(:,:,i),scale*PlotScale);
            figure(fnum);
            param3d1(axisX,axisY,axisZ); 
            h=gce();
        
            axis_x = h.children(3)
            axis_y = h.children(2)
            axis_z = h.children(1)
            
            if i==1 then   // base is black 
                axis_x.foreground = 1;
                axis_y.foreground = 1;
                axis_z.foreground = 1;
                font_color = 1;
            elseif i==nf1    // end is violet
                axis_x.foreground = 6;
                axis_y.foreground = 6;
                axis_z.foreground = 6;
                font_color = 6;                
            else       // others are red                     
                axis_x.foreground = 5;
                axis_y.foreground = 5;
                axis_z.foreground = 5;
                font_color = 5;
            end
            if i== missingframe then
 
                axis_x.line_style = 8;    // dotted
                axis_y.line_style = 8;
                axis_z.line_style = 8;

            end
            
     
            axis_x.polyline_style = 4;
            axis_y.polyline_style = 4;
            axis_z.polyline_style = 4;

            axis_x.thickness = 1;
            axis_y.thickness = 1;
            axis_z.thickness = 1;

            // place axis label to figure
            xttext = sprintf("X%s",kc1.Frame(i).name);
            yttext = sprintf("Y%s",kc1.Frame(i).name);
            zttext = sprintf("Z%s",kc1.Frame(i).name);

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
            if i==missingframe then
            end
            // place frame name and location at origin
            if ShowOrigin 
                otext = sprintf("{%s} (%4.2f,%4.2f,%4.2f)",kc.Frame(i).name,d1(1,i),d1(2,i),d1(3,i));
            else
                otext = sprintf("{%s}",kc1.Frame(i).name);
            end
            xstring(0.5,0.5,otext);
            opos = get("hdl");
            opos.font_foreground = font_color;
            opos.data = [d1(1,i), d1(2,i), d1(3,i)];

            if i<nf1
                // draw line between frame
                Xl = [d1(1,i);d1(1,i+1)];                  
                Yl = [d1(2,i); d1(2,i+1)]; 
                Zl = [d1(3,i); d1(3,i+1)];                    
                figure(fnum);
                param3d(Xl,Yl,Zl);
                hl=gce();
                hl.foreground = 5;
                if i==missingframe-1 then
                    hl.foreground = 3;  // green
                    hl.line_style = 8;  // dotted
                    //hl.thickness = 2;
                else
                    //hl.foreground = 8;  // white
                    hl.line_style = 1;  // solid
                    //hl.thickness = 1;
                end
                hl.polyline_style = 4;              
                hl.thickness = 1;
            end

          // pause;
    end    // for i=1:nf1
    
    // =============== Plot chain 2 =======================
    for i=1:nf2    // create plot data for each frame in chain 2 
                    
        if i>1 & i<nf2 then // omit  first and last frame since already plotted in chain 1
             is = modulo(i,6);
             scale = ScaleVec(is);

            [axisX,axisY,axisZ]=GeoMakeFrame(d2(:,i)',R2(:,:,i),scale*PlotScale);
            figure(fnum);
            param3d1(axisX,axisY,axisZ); 
            h=gce();
        
            axis_x = h.children(3)
            axis_y = h.children(2)
            axis_z = h.children(1)
            
                   
            axis_x.foreground = 2;
            axis_y.foreground = 2;
            axis_z.foreground = 2;
            font_color = 2;
 
     
            axis_x.polyline_style = 4;
            axis_y.polyline_style = 4;
            axis_z.polyline_style = 4;
            axis_x.thickness = 1;
            axis_y.thickness = 1;
            axis_z.thickness = 1;

            // place axis label to figure
            xttext = sprintf("X%s",kc2.Frame(i).name);
            yttext = sprintf("Y%s",kc2.Frame(i).name);
            zttext = sprintf("Z%s",kc2.Frame(i).name);

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
            // place frame name and location at origin

            
                if ShowOrigin 
                    otext = sprintf("{%s} (%4.2f,%4.2f,%4.2f)",kc.Frame(i).name,d2(1,i),d2(2,i),d2(3,i));
                else
                    otext = sprintf("{%s}",kc2.Frame(i).name);
                end
          
                xstring(0.5,0.5,otext);
                opos = get("hdl");
                opos.font_foreground = 2;
                opos.data = [d2(1,i), d2(2,i), d2(3,i)];
            end
            
            if i<nf2
                // draw line between frame
                Xl = [d2(1,i);d2(1,i+1)];                  
                Yl = [d2(2,i); d2(2,i+1)]; 
                Zl = [d2(3,i); d2(3,i+1)];                    
                figure(fnum);
                param3d(Xl,Yl,Zl);
                hl=gce();
    
                hl.polyline_style = 4;
                
                hl.foreground = 2;
                hl.thickness = 1;
            end

      
    end    // for i=1:nf2

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

    h_axes.rotation_angles = [63.5,26];

    h_axes.data_bounds = [xmin,ymin,zmin; xmax,ymax,zmax];
 
    if kc1.name ~= '' & kc2.name ~= '' then 
        msg=sprintf("Closed chain of %s and %s",kc1.name,kc2.name);
        title(msg);
    end
    if ShowGrid then xgrid;
    end
    if missingframe~= 0 then
       
        printf("\n\n Missing data T: {%s} w.r.t {%s} in %s is computed as \n",kc1.Frame(missingframe).name,kc1.Frame(missingframe-1).name,kc1.name);
        disp(T_rel);
        printf("\n\n");
    end
endfunction    

function PlotResolveFrameHelp()
        printf("=============================================================\n");    
        printf("Usage: [T_rel,T_abs,kc1]=PlotResolveFrame(kc1, kc2, <options>)\n\n");
        printf("\twhere kc1 is an incomplete kinematic chain \n");
        printf("\t kc2 must be a completed kinematic chain with same start and end as kc1\n");
        printf("\tOr, kc2 could simply be a homogeneous matrix describing end frame of kc1 w.r.t base\n");
        printf("Available options: (put string in quotes)\n");
        printf("\tgrid: add grid to plot\n");
        printf("\t<figure, i>: plot on window number i\n");
        printf("\toinfo: show origin coordinates\n");
        printf("=============================================================\n");        
endfunction        