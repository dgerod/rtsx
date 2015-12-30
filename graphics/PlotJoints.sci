// =====================================================================
// PlotRobot.sci sketch a robot manipulator in 3-D
// www.controlsystemslab.com   July 2012
// =====================================================================

    //Ti(:,:,1)=A(:,:,1);
    //Ri(:,:,1)=Ti(1:3,1:3,1);    // orientation of joint 1
    //di(:,1)=Ti(1:3,4,1);        // location of joint 1

// ---------------------------------------------------------------------

function [Hdls,Hdlb] = plotJointR (Radius,Height,di,Ri,SideCount)

    vertexData = GeoVerMakeCylinder(di,Ri,Radius,Height,SideCount);
    [Xs,Ys,Zs,Xb,Yb,Zb] = GeoPatMakeCylinder(vertexData);

    // Draw side patches
    plot3d(Xs,Ys,Zs);
    Hdls = gce();
    Hdls.color_mode = 4;
    Hdls.foreground = 1;
    Hdls.hiddencolor = 5;

    // Draw bottom patches
    plot3d(Xb,Yb,Zb);
    Hdlb = gce();
    Hdlb.color_mode = 4;
    Hdlb.foreground = 1;
    Hdlb.hiddencolor = 5;

    return []
    
endfunction

// ---------------------------------------------------------------------

function [Hdl] = plotJointP (L,di,Ri)

    Hdl =[];

    vertexData = GeoVerMakeBlock(di,Ri,L);
    [X,Y,Z] = GeoPatMakeBlock(vertexData);
    
    plot3d(X,Y,Z);
    Hdl = gce();
    Hdl.color_mode = 4;
    Hdl.foreground = 1;
    Hdl.hiddencolor = 4;
    
endfunction

// ---------------------------------------------------------------------

PlotJointR = plotJointR;
PlotJointP = plotJointP;

// =====================================================================
