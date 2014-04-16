//PlotFrame.sce   test plotting coordinate frame
xdel(winsid());
loc = [-1 -1 -1];
R = rotx(%pi/4);
alength = 1;
[X,Y,Z]=GeoMakeFrame(loc,R,alength);
figure(1);
param3d1(X,Y,Z);
h=gce();
axis_x = h.children(3);
axis_y = h.children(2);
axis_z = h.children(1);
axis_x.foreground = 5;
axis_y.foreground = 5;
axis_z.foreground = 5;
axis_x.polyline_style = 4;
axis_y.polyline_style = 4;
axis_z.polyline_style = 4;
axis_x.thickness = 2;
axis_y.thickness = 2;
axis_z.thickness = 2;

        xstring(0.5,0.5,'X');
        xpos = get("hdl");  // get the handle of the newly created object
        // Now set actual position
        xpos.data = [X(2,1),Y(2,1),Z(2,1)];

        xstring(0.5,0.5,'Y');
        ypos = get("hdl");
        ypos.data = [X(2,2),Y(2,2),Z(2,2)];

        xstring(0.5,0.5,'Z');
        zpos = get("hdl");
        zpos.data = [X(2,3),Y(2,3),Z(2,3)];

xmin = loc(1)-alength-0.1;
xmax = loc(1)+alength+0.1;
ymin = loc(2)-alength-0.1;
ymax = loc(2)+alength+0.1;
zmin = loc(3)-alength-0.1;
zmax = loc(3)+alength+0.1;


    // Axes settings
    xlabel("x",'fontsize',2);
    ylabel("y",'fontsize',2);
    zlabel("z",'fontsize',2);
    h_axes = gca();
    h_axes.font_size = 2;
    h_axes.isoview = "on";
    h_axes.box = "off";
    h_axes.rotation_angles = [63.5,-127];
    h_axes.data_bounds = [xmin,ymin,zmin; xmax,ymax,zmax];
    xgrid;