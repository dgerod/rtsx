// TRPLOT.SCI  plot a coordiate frame that corresponds to a rotation/homogeneous
// transformation matrix
// www.controlsystemslab.com   July 2012
// Update list:
// Sep 3, 2012: add quaternion input option

function trplot(T,varargin)
    ShowGrid = 0;  // default plot to no grid
    ShowWorld = 0;  // default no world coordinate
    Hold = 0;        // hold previous figures
    ShowOrigin = 0;  // default to no origin info
    axscale = 0;   // axscale can be given or select automatically
    // retrieve variable arguments
    fnum = max(winsid())+1;
    setfignum = 0;     // flag whether user set figure number
    framenum = 1;   // frame number, default to 1
    fcolor = 5;         // frame color. Default to 'red'
    lstyle = 1;        // line style. Default to solid
    varnum=length(varargin);  // number of arguments
    
    for iv =1:varnum
        if varargin(iv)=='grid' then
            ShowGrid = 1;
        end
        if varargin(iv)=='world' then
            ShowWorld = 1;
        end
        if varargin(iv)=='origin' then
            ShowOrigin = 1;
        end        
        if varargin(iv)=='hold' then
            Hold = 1;
            if ~setfignum then  // hold last window
                curwin = winsid();
                if curwin~=[] fnum = curwin(length(curwin));
                end
            end
        end
        if varargin(iv)=='figure'& (iv<varnum) then  // can put figure number as arg
            fnum = varargin(iv+1); 
            setfignum = 1;  // user has set figure number
        end
        if varargin(iv)=='frame' & (iv<varnum) then
            framenum = varargin(iv+1);
        end        
        if varargin(iv)=='axscale' & (iv<varnum) then
            axscale = varargin(iv+1);
        end 
        if varargin(iv)=='color' & (iv<varnum) then
            fcolor = colormap(varargin(iv+1));
        end 
        if varargin(iv)=='linestyle' & (iv<varnum) then
            lstyle = linesmap(varargin(iv+1));   
        end
    end
    if find(winsid()==fnum) & ~Hold then           
        xdel(fnum);
    end
    
    if isquaternion(T)     // accept Quaternion argument
       T = q2tr(T);    
    end
    d = size(T);
    p = zeros(3,1);   // position of origin
    R = zeros(3,3);    // rotation matrix
    if  length(d)> 2 
        error("Cannot accept 3-D matrix")
    end
    select d
        case[3 1]    // only position vector passed
            p = T;   
            R = eye(3,3); 
        case [1 3] 
            p = T';
            R = eye(3,3);
        case [3,3]    // only rotation matrix passed
            R = T;
            p = [0 0 0]';    // origin at 0,0,0
        case [4,4]
            R = T(1:3,1:3);
            p = T(1:3,4);
        else
            error("Bad matrix dimension");
    end
plen = norm(p);
if plen==0 then plen = 1;  // cover when origin at (0,0,0)
end
ScaleVec = [0.2 0.25 0.35 0.4 0.45 0.5 0.55 0.6];    // axis length is picked relative to color
                                    // to avoid printing axis label on top of each other
if axscale == 0
   axscale = ScaleVec(fcolor);
end
waxlen = 0.3*plen;  // world axis length 
axlen = axscale*plen;   
// adjust X,Y,Z axis range accordingly
bmargin = 0.65*plen;
if p(1)>=0  // positive x
    xmin = -bmargin;
    xmax = p(1)+bmargin;
else 
    xmin = p(1)-bmargin;
    xmax = bmargin;
end
if p(2)>=0  // positive y
    ymin = -bmargin;
    ymax = p(2)+bmargin;
else 
    ymin = p(2)-bmargin;
    ymax = bmargin;
end
if p(3)>=0  // positive z
    zmin = -bmargin;
    zmax = p(3)+bmargin;
else 
    zmin = p(3)-bmargin;
    zmax = bmargin;
end


// plot world coordinate
if ShowWorld then
        [axisX0,axisY0,axisZ0]=GeoMakeFrame([0 0 0],eye(3,3),waxlen);
        figure(fnum);
        param3d1(axisX0,axisY0,axisZ0); 
        h=gce();
        
        axis_x0 = h.children(3)
        axis_y0 = h.children(2)
        axis_z0 = h.children(1)
        axis_x0.foreground = 2;
        axis_y0.foreground = 2;
        axis_z0.foreground = 2;
        axis_x0.polyline_style = 4;
        axis_y0.polyline_style = 4;
        axis_z0.polyline_style = 4;
        // show world frame as dotted
        axis_x0.line_style = 8; 
        axis_y0.line_style = 8;
        axis_z0.line_style = 8;

        axis_x0.thickness = 1;
        axis_y0.thickness = 1;
        axis_z0.thickness = 1;

        // place axis label to figure

        xwtext = "X0";
        ywtext = "Y0";
        zwtext = "Z0";
        xstring(0.5,0.5,xwtext);
        xwpos = get("hdl");  // get the handle of the newly created object
        
        // Now set actual position
        xwpos.data = [axisX0(2,1),axisY0(2,1),axisZ0(2,1)];
        
        xstring(0.5,0.5,ywtext);
        ywpos = get("hdl");
        ywpos.data = [axisX0(2,2),axisY0(2,2),axisZ0(2,2)];

        xstring(0.5,0.5,zwtext);
        zwpos = get("hdl");
        zwpos.data = [axisX0(2,3),axisY0(2,3),axisZ0(2,3)];
        if ShowOrigin
            owtext ="(0,0,0)";
            xstring(0.5,0.5,owtext);
            owpos = get("hdl");
            owpos.data = [0,0,0];
        end
end

// now plot the coordinate frame
[axisX,axisY,axisZ]=GeoMakeFrame(p',R,axlen);
figure(fnum);
param3d1(axisX,axisY,axisZ); 
h=gce();

axis_x = h.children(3)
axis_y = h.children(2)
axis_z = h.children(1)
axis_x.foreground = fcolor;
axis_y.foreground = fcolor;
axis_z.foreground = fcolor;
axis_x.polyline_style = 4;
axis_y.polyline_style = 4;
axis_z.polyline_style = 4;

axis_x.line_style = lstyle;
axis_y.line_style = lstyle;
axis_z.line_style = lstyle;

axis_x.thickness = 1;
axis_y.thickness = 1;
axis_z.thickness = 1;

// place axis label to figure
if type(framenum)==1  // number
    xtext = sprintf("X%d", framenum);
    ytext = sprintf("Y%d", framenum);
    ztext = sprintf("Z%d", framenum);
elseif type(framenum)==10  // string
    xtext = sprintf("X%s", framenum);
    ytext = sprintf("Y%s", framenum);
    ztext = sprintf("Z%s", framenum);
end
xstring(0.5,0.5,xtext);
xpos = get("hdl");  // get the handle of the newly created object
// Now set actual position
xpos.font_foreground = fcolor;
xpos.data = [axisX(2,1),axisY(2,1),axisZ(2,1)];

xstring(0.5,0.5,ytext);
ypos = get("hdl");
ypos.font_foreground = fcolor;
ypos.data = [axisX(2,2),axisY(2,2),axisZ(2,2)];

xstring(0.5,0.5,ztext);
zpos = get("hdl");
zpos.font_foreground = fcolor;
zpos.data = [axisX(2,3),axisY(2,3),axisZ(2,3)];

// show origin if option set
if ShowOrigin
    otext =sprintf("(%d,%d,%d)",p(1),p(2),p(3));
    xstring(0.5,0.5,otext);
    opos = get("hdl");
    opos.font_foreground = fcolor;
    opos.data = p';
end
// Axes settings
xlabel("x",'fontsize',2);
ylabel("y",'fontsize',2);
zlabel("z",'fontsize',2);
h_axes = gca();
h_axes.font_size = 2;
h_axes.isoview = "on";
h_axes.box = "off";
h_axes.rotation_angles = [60,30];
h_axes.data_bounds = [xmin,ymin,zmin; xmax,ymax,zmax];
if ShowGrid then xgrid;
end
endfunction

