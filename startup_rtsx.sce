// startup_RTSX.sce  
// www.controlsystemslab.com   Feb 2014
// script file to load all RTSX functions

printf("\nRobotic Tools for Scilab/Xcos (RTSX) Version 1.00\n");
printf("by Control Systems Lab,  February 2014\n");
printf("http://www.controlsystemslab.com/rtsx\n");

funcprot(0);  // suppress warning when function is redefined

// Get root path of the toolbox
RTSX_ROOT = get_absolute_file_path( "startup_rtsx.sce" );

// ============== common constants ================
pi = %pi;
false = 0;
true = 1;
// boolean
False = (1==0);
True = (1==1);
// ================= math functions ================
// quaternion
exec(RTSX_ROOT + '/quaternion/Quaternion.sci',-1);
exec(RTSX_ROOT + '/quaternion/q2tr.sci',-1);
exec(RTSX_ROOT + '/quaternion/q2vec.sci',-1);
exec(RTSX_ROOT + '/quaternion/tr2q.sci',-1);
exec(RTSX_ROOT + '/quaternion/isquaternion.sci',-1);
exec(RTSX_ROOT + '/quaternion/q2str.sci',-1);  // convert to string
exec(RTSX_ROOT + '/quaternion/qinv.sci',-1);  // quaternion inversion
exec(RTSX_ROOT + '/quaternion/qnorm.sci',-1);
exec(RTSX_ROOT + '/quaternion/qunit.sci',-1);
exec(RTSX_ROOT + '/quaternion/isqequal.sci',-1);
exec(RTSX_ROOT + '/quaternion/qadd.sci',-1);      // addition/subtraction
exec(RTSX_ROOT + '/quaternion/qmult.sci',-1);     // multiplication
exec(RTSX_ROOT + '/quaternion/qpower.sci',-1);
exec(RTSX_ROOT + '/quaternion/qdivide.sci',-1);
exec(RTSX_ROOT + '/quaternion/qinterp.sci',-1);
exec(RTSX_ROOT + '/quaternion/qscale.sci',-1);

// ===============rotation matrices================
// basic rotation matrices
exec(RTSX_ROOT + '/rotx.sci',-1);
exec(RTSX_ROOT + '/roty.sci',-1);
exec(RTSX_ROOT + '/rotz.sci',-1);
// data conversions
exec(RTSX_ROOT + '/eul2r.sci',-1);    // euler angles to rotation matrix R
exec(RTSX_ROOT + '/rpy2r.sci',-1);    // RPY angles to R
exec(RTSX_ROOT + '/angvec2r.sci',-1);    // angle/vector to R
exec(RTSX_ROOT + '/rotxyz2r.sci',-1);  // axis X,Y,Z rotations to R

// ================= homogeneous transformation ===========
// rotation
exec(RTSX_ROOT + '/trotx.sci',-1);
exec(RTSX_ROOT + '/troty.sci',-1);
exec(RTSX_ROOT + '/trotz.sci',-1);
exec(RTSX_ROOT + '/trotw.sci',-1);  // rotation about a world coordinate axis
// translation
exec(RTSX_ROOT + '/transl.sci',-1);
exec(RTSX_ROOT + '/mtransl.sci',-1);    // translation sequences 
// data conversions
exec(RTSX_ROOT + '/t2p.sci',-1);
exec(RTSX_ROOT + '/t2r.sci',-1);
exec(RTSX_ROOT + '/r2t.sci',-1);
exec(RTSX_ROOT + '/rp2t.sci',-1);
exec(RTSX_ROOT + '/eul2t.sci',-1);
exec(RTSX_ROOT + '/rpy2t.sci',-1);
exec(RTSX_ROOT + '/tr2eul.sci',-1);
exec(RTSX_ROOT + '/tr2angvec.sci',-1);
exec(RTSX_ROOT + '/angvec2t.sci',-1);
exec(RTSX_ROOT + '/tr2rpy.sci',-1);
exec(RTSX_ROOT + '/gentrot.sci',-1); // generate sequence of basic rotation T
// frame construction and manipulation
exec(RTSX_ROOT + '/Frame.sci',-1);
exec(RTSX_ROOT + '/SerialFrame.sci',-1);
exec(RTSX_ROOT + '/InsertFrame.sci',-1);
exec(RTSX_ROOT + '/DeleteFrame.sci',-1);
exec(RTSX_ROOT + '/ReplaceFrame.sci',-1);
exec(RTSX_ROOT + '/ResolveFrame.sci',-1);
// utilities
exec(RTSX_ROOT + '/trnorm.sci',-1);
// ============ robot modeling and forward/inverse kinematics =====================
exec(RTSX_ROOT + '/Link.sci',-1);
exec(RTSX_ROOT + '/AppendLink.sci',-1);    // append a robot link
exec(RTSX_ROOT + '/RemoveLink.sci',-1);    // remove a robot link
exec(RTSX_ROOT + '/ReplaceLink.sci',-1);    // replace a robot link
exec(RTSX_ROOT + '/AttachBase.sci',-1);
exec(RTSX_ROOT + '/AttachTool.sci',-1);
exec(RTSX_ROOT + '/DetachBase.sci',-1);
exec(RTSX_ROOT + '/DetachTool.sci',-1);
exec(RTSX_ROOT + '/Link2AT.sci',-1);
exec(RTSX_ROOT + '/Robot2AT.sci',-1);    // added Nov 2012
exec(RTSX_ROOT + '/SerialLink.sci',-1);
exec(RTSX_ROOT + '/Robotinfo.sci',-1);
exec(RTSX_ROOT + '/FKine.sci',-1);
exec(RTSX_ROOT + '/Robot2hAT.sci',-1);
exec(RTSX_ROOT + '/UpdateRobot.sci',-1);
exec(RTSX_ROOT + '/UpdateRobotLink.sci',-1);
exec(RTSX_ROOT + '/IKine6s.sci',-1);
exec(RTSX_ROOT + '/ikine.sci',-1);

// ======================= velocity kinematics ============================
exec(RTSX_ROOT + '/tr2delta.sci',-1);
exec(RTSX_ROOT + '/delta2tr.sci',-1);
exec(RTSX_ROOT + '/tr2jac.sci',-1);
exec(RTSX_ROOT + '/jacobn.sci',-1);
exec(RTSX_ROOT + '/jacob0.sci',-1);
exec(RTSX_ROOT + '/rpy2jac.sci',-1);
exec(RTSX_ROOT + '/eul2jac.sci',-1);
exec(RTSX_ROOT + '/maniplty.sci',-1);    // added Nov 2012

// ======================== plotting/amimation functions ==================

exec(RTSX_ROOT + '/trplot.sci',-1);
exec(RTSX_ROOT + '/tranimate.sci',-1);
exec(RTSX_ROOT + '/PlotFrame.sci',-1);
exec(RTSX_ROOT + '/PlotResolveFrame.sci',-1);
exec(RTSX_ROOT + '/PlotRobot.sci',-1);
exec(RTSX_ROOT + '/PlotRobotFrame.sci',-1);
exec(RTSX_ROOT + '/AnimateRobot.sci',-1);
exec(RTSX_ROOT + '/AnimateRobotFrame.sci',-1);

// =================== path generation ==================
exec(RTSX_ROOT + '/cpoly.sci',-1);  // cubic polynomial
exec(RTSX_ROOT + '/qpoly.sci',-1);  // quintic polynomial
exec(RTSX_ROOT + '/lspb.sci',-1);   // linear segment parabolic blend
exec(RTSX_ROOT + '/mtraj.sci',-1);
exec(RTSX_ROOT + '/mstraj.sci',-1);
exec(RTSX_ROOT + '/jtraj.sci',-1);
exec(RTSX_ROOT + '/trinterp.sci',-1);
exec(RTSX_ROOT + '/ctraj.sci',-1);

// ================= dynamics ========================
exec(RTSX_ROOT + '/friction.sci',-1);
exec(RTSX_ROOT + '/rne.sci',-1);
exec(RTSX_ROOT + '/rne_dh.sci',-1);
exec(RTSX_ROOT + '/gravload.sci',-1);
exec(RTSX_ROOT + '/inertia.sci',-1);
exec(RTSX_ROOT + '/coriolis.sci',-1);
exec(RTSX_ROOT + '/payload.sci',-1);
exec(RTSX_ROOT + '/accel.sci',-1);

// ================ Machine vision ==============================
// ============  first added April, 2013 =============================
exec(RTSX_ROOT + '/vision/CentralCamera.sci',-1);
exec(RTSX_ROOT + '/vision/CamProject.sci',-1);
exec(RTSX_ROOT + '/vision/CamPlot.sci',-1);
exec(RTSX_ROOT + '/vision/mkgrid.sci',-1);
exec(RTSX_ROOT + '/vision/utilities.sci',-1);
exec(RTSX_ROOT + '/vision/visjac_p.sci',-1);
exec(RTSX_ROOT + '/vision/IBVS4.sci',-1);
exec(RTSX_ROOT + '/vision/depth_estimator.sci',-1);

// ==================== graphic functions ========================
exec(RTSX_ROOT + '/graphics/GeoPatMakeBlock.sci',-1);  // make patches for rectangular block
exec(RTSX_ROOT + '/graphics/GeoVerMakeBlock.sci',-1); // make vertices of rectangular block
exec(RTSX_ROOT + '/graphics/GeoPatMakeCylinder.sci',-1);
exec(RTSX_ROOT + '/graphics/GeoVerMakeCylinder.sci',-1);
exec(RTSX_ROOT + '/graphics/GeoMakeJointAxis.sci',-1);
exec(RTSX_ROOT + '/graphics/GeoMakeFrame.sci',-1);
exec(RTSX_ROOT + '/graphics/SetViewAngle.sci',-1);
exec(RTSX_ROOT + '/graphics/GetViewAngle.sci',-1);
exec(RTSX_ROOT + '/graphics/colormap.sci',-1);
exec(RTSX_ROOT + '/graphics/linesmap.sci',-1);
exec(RTSX_ROOT + '/graphics/ellipsoid.sci',-1);   // plot ellipsoid, added Nov 10,2012
exec(RTSX_ROOT + '/graphics/plot_ellipse.sci',-1);
// =================== common functions ==========================
exec(RTSX_ROOT + '/common/numrows.sci',-1);
exec(RTSX_ROOT + '/common/numcols.sci',-1);
exec(RTSX_ROOT + '/common/hold.sci',-1);
exec(RTSX_ROOT + '/common/linecenter.sci',-1);
exec(RTSX_ROOT + '/common/unit.sci',-1);
exec(RTSX_ROOT + '/common/isrot.sci',-1);
exec(RTSX_ROOT + '/common/isvec.sci',-1);
exec(RTSX_ROOT + '/common/ishomog.sci',-1);
exec(RTSX_ROOT + '/common/Xlocate.sci',-1);  // locate intesection of X_i and Z_i-1
exec(RTSX_ROOT + '/common/Xlocate4.sci',-1); // modified for 4-D A and T matrices
exec(RTSX_ROOT + '/common/isspherical.sci',-1);  // test spherical wrist
exec(RTSX_ROOT + '/common/vex.sci',-1);
exec(RTSX_ROOT + '/common/skew.sci',-1);
exec(RTSX_ROOT + '/common/blkdiag2.sci',-1);
exec(RTSX_ROOT + '/common/isscalar.sci',-1);
exec(RTSX_ROOT + '/common/polyval.sci',-1);
exec(RTSX_ROOT + '/common/_zeros_ones.sci',-1);
exec(RTSX_ROOT + '/common/unitvec.sci',-1);   // computes unit vector
exec(RTSX_ROOT + '/common/display.sci',-1);
exec(RTSX_ROOT + '/common/cross.sci',-1);  // cross product
// ============= demo ===============
exec(RTSX_ROOT + '/demo/rprdemo.sci',-1);






















