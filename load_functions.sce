// =======================================================================================
// load_functions.sce
// ---------------------------------------------------------------------------------------
// File to load all RTSX functions
// =======================================================================================

exec(RTSX_ROOT + 'common/loader.sce');
exec(RTSX_ROOT + 'quaternion/loader.sce');
exec(RTSX_ROOT + 'transformation/loader.sce');
exec(RTSX_ROOT + 'kinematics/loader.sce');
exec(RTSX_ROOT + 'dynamics/loader.sce');
exec(RTSX_ROOT + 'trajectories/loader.sce');
exec(RTSX_ROOT + 'robot/loader.sce');
exec(RTSX_ROOT + 'graphics/loader.sce');

if load3rdPartyOn == %t then
    exec(RTSX_ROOT + 'aux3rdparty/loader.sce',-1);
end

// =======================================================================================

