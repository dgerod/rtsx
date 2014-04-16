//PlotCylinder.sce   plot a cylinder (for testing purpose)
// www.controlsystemslab.com   July 2012
xdel(winsid());
R = rotx(%pi/4);        // orientation
//R = eye(3,3);
o = [0 0 0];        // origin
// cylinder specs
Radius = 0.1;
Height = 0.3;
SideCount = 10;

VertexData = GeoVerMakeCylinder(o,R,Radius,Height,SideCount);
[Xs,Ys,Zs,Xb,Yb,Zb] = GeoPatMakeCylinder(VertexData);

// Draw side patches
figure(1);
plot3d(Xs,Ys,Zs);
h_fac3d = gce();
h_fac3d.color_mode = 4;
h_fac3d.foreground = 1;
h_fac3d.hiddencolor = 5;

// Draw bottom patches
figure(1);
plot3d(Xb,Yb,Zb);
h2_fac3d = gce();
h2_fac3d.color_mode = 4;
h2_fac3d.foreground = 1;
h2_fac3d.hiddencolor = 5;

// Axes settings
xlabel("x",'fontsize',2);
ylabel("y",'fontsize',2);
zlabel("z",'fontsize',2);
h_axes = gca();
h_axes.font_size = 2;
h_axes.isoview = "on";
h_axes.box = "off";
h_axes.rotation_angles = [63.5,-127];
h_axes.data_bounds = [-1,-1,-1; 1,1,1];
xgrid;