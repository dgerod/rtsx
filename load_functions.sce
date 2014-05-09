// =======================================================================================
// load_functions.sce
// ---------------------------------------------------------------------------------------
// File to load all RTSX functions
// =======================================================================================

// ================= math functions ================
// quaternion
exec(RTSX_ROOT + 'quaternion/Quaternion.sci',-1);
exec(RTSX_ROOT + 'quaternion/q2tr.sci',-1);
exec(RTSX_ROOT + 'quaternion/q2vec.sci',-1);
exec(RTSX_ROOT + 'quaternion/tr2q.sci',-1);
exec(RTSX_ROOT + 'quaternion/isquaternion.sci',-1);
exec(RTSX_ROOT + 'quaternion/q2str.sci',-1);  // convert to string
exec(RTSX_ROOT + 'quaternion/qinv.sci',-1);  // quaternion inversion
exec(RTSX_ROOT + 'quaternion/qnorm.sci',-1);
exec(RTSX_ROOT + 'quaternion/qunit.sci',-1);
exec(RTSX_ROOT + 'quaternion/isqequal.sci',-1);
exec(RTSX_ROOT + 'quaternion/qadd.sci',-1);      // addition/subtraction
exec(RTSX_ROOT + 'quaternion/qmult.sci',-1);     // multiplication
exec(RTSX_ROOT + 'quaternion/qpower.sci',-1);
exec(RTSX_ROOT + 'quaternion/qdivide.sci',-1);
exec(RTSX_ROOT + 'quaternion/qinterp.sci',-1);exec(RTSX_ROOT + 'quaternion/qscale.sci',-1);

// ===============rotation matrices================
exec(RTSX_ROOT + 'transformation/loader.sce');

// ============ robot modeling and forward/inverse kinematics =====================
// frame construction and manipulation
exec(RTSX_ROOT + 'Frame.sci',-1);
exec(RTSX_ROOT + 'SerialFrame.sci',-1);
exec(RTSX_ROOT + 'InsertFrame.sci',-1);
exec(RTSX_ROOT + 'DeleteFrame.sci',-1);
exec(RTSX_ROOT + 'ReplaceFrame.sci',-1);
exec(RTSX_ROOT + 'ResolveFrame.sci',-1);
// robot modeling
exec(RTSX_ROOT + 'Link.sci',-1);
exec(RTSX_ROOT + 'AppendLink.sci',-1);    // append a robot link
exec(RTSX_ROOT + 'RemoveLink.sci',-1);    // remove a robot link
exec(RTSX_ROOT + 'ReplaceLink.sci',-1);    // replace a robot link
exec(RTSX_ROOT + 'AttachBase.sci',-1);
exec(RTSX_ROOT + 'AttachTool.sci',-1);
exec(RTSX_ROOT + 'DetachBase.sci',-1);
exec(RTSX_ROOT + 'DetachTool.sci',-1);
exec(RTSX_ROOT + 'Link2AT.sci',-1);
exec(RTSX_ROOT + 'Robot2AT.sci',-1);    // added Nov 2012
exec(RTSX_ROOT + 'SerialLink.sci',-1);
exec(RTSX_ROOT + 'RobotInfo.sci',-1);
exec(RTSX_ROOT + 'Robot2hAT.sci',-1);
exec(RTSX_ROOT + 'UpdateRobot.sci',-1);
exec(RTSX_ROOT + 'UpdateRobotLink.sci',-1);
exec(RTSX_ROOT + 'maniplty.sci',-1);    // added Nov 2012

// ======================== plotting/amimation functions ==================
exec(RTSX_ROOT + 'graphics/loader.sce');

// =================== path generation ==================
exec(RTSX_ROOT + 'trajectories/loader.sce');

// ===============kinematics================
exec(RTSX_ROOT + 'kinematics/loader.sce');

// ===============dynamics================
exec(RTSX_ROOT + 'dynamics/loader.sce');
exec(RTSX_ROOT + 'friction.sci',-1);
exec(RTSX_ROOT + 'gravload.sci',-1);
exec(RTSX_ROOT + 'inertia.sci',-1);
exec(RTSX_ROOT + 'payload.sci',-1);

// ================ Machine vision ==============================
// ============  first added April, 2013 =============================
exec(RTSX_ROOT + 'vision/CentralCamera.sci',-1);
exec(RTSX_ROOT + 'vision/CamProject.sci',-1);
exec(RTSX_ROOT + 'vision/CamPlot.sci',-1);
exec(RTSX_ROOT + 'vision/mkgrid.sci',-1);
exec(RTSX_ROOT + 'vision/utilities.sci',-1);
exec(RTSX_ROOT + 'vision/visjac_p.sci',-1);
exec(RTSX_ROOT + 'vision/IBVS4.sci',-1);
exec(RTSX_ROOT + 'vision/depth_estimator.sci',-1);

// =================== common functions ==========================
exec(RTSX_ROOT + 'common/numrows.sci',-1);
exec(RTSX_ROOT + 'common/numcols.sci',-1);
exec(RTSX_ROOT + 'common/hold.sci',-1);
exec(RTSX_ROOT + 'common/linecenter.sci',-1);
exec(RTSX_ROOT + 'common/unit.sci',-1);
exec(RTSX_ROOT + 'common/isrot.sci',-1);
exec(RTSX_ROOT + 'common/isvec.sci',-1);
exec(RTSX_ROOT + 'common/ishomog.sci',-1);
exec(RTSX_ROOT + 'common/Xlocate.sci',-1);  // locate intesection of X_i and Z_i-1
exec(RTSX_ROOT + 'common/Xlocate4.sci',-1); // modified for 4-D A and T matrices
exec(RTSX_ROOT + 'common/isspherical.sci',-1);  // test spherical wrist
exec(RTSX_ROOT + 'common/vex.sci',-1);
exec(RTSX_ROOT + 'common/skew.sci',-1);
exec(RTSX_ROOT + 'common/blkdiag2.sci',-1);
exec(RTSX_ROOT + 'common/isscalar.sci',-1);
exec(RTSX_ROOT + 'common/polyval.sci',-1);
exec(RTSX_ROOT + 'common/_zeros_ones.sci',-1);
exec(RTSX_ROOT + 'common/unitvec.sci',-1);   // computes unit vector
exec(RTSX_ROOT + 'common/display.sci',-1);
exec(RTSX_ROOT + 'common/cross.sci',-1);  // cross product

// =================== 3rd party functions ==========================
if load3rdPartyOn == %t then
    exec(RTSX_ROOT + '3rdparty/loader.sce',-1);
end

// =======================================================================================

