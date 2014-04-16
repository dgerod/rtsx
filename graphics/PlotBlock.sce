// PlotBlock.sce   plot a rectangular block (for testing purpose)
// www.controlsystemslab.com   July 2012
xdel(winsid());
//R = eye(3,3);        // orientation
R = rotx(%pi/4);
o = [0 0 0];        // origin
// Block specification
Lx = 0.1;
Ly = 0.1;
Lz = 0.1;

VertexData = GeoVerMakeBlock(o,R,[Lx,Ly,Lz]);
[X,Y,Z] = GeoPatMakeBlock(VertexData);

figure(1);
plot3d(X,Y,Z);
h_fac3d = gce();
h_fac3d.color_mode = 4;
h_fac3d.foreground = 1;
h_fac3d.hiddencolor = 4;

// Axes settings
xlabel("x",'fontsize',2);
ylabel("y",'fontsize',2);
zlabel("z",'fontsize',2);
h_axes = gca();
h_axes.font_size = 2;
h_axes.isoview = "on";
h_axes.box = "off";
h_axes.rotation_angles = [63.5,-127];
h_axes.data_bounds = [-0.5,-0.5,-0.5;0.5,0.5,0.5];
xgrid;