// =======================================================================================
// loader.sce
// =======================================================================================

pathName = get_absolute_file_path("loader.sce");

// Load general functions
getd(pathName);

// Load specific kinematics
exec(pathName + "delta-2_robot/loader.sce", -1) 
exec(pathName + "delta-3_robot/loader.sce", -1) 
if load3rdPartyOn == %t then
    exec(pathName + "barret_wam_7dof/loader.sce", -1) 
end
exec(pathName + "scara/loader.sce", -1) 
// =======================================================================================

