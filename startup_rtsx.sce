// =======================================================================================
// startup_rtsx.sce  
// www.controlsystemslab.com   Feb 2014
// ---------------------------------------------------------------------------------------
// Script that initialize the RTSX toolbox
// =======================================================================================

mprintf("\n");
mprintf("=====================================\n");
mprintf("Robotics Tools for Scilab/Xcos (RTSX)\n");
mprintf("       Control Systems Lab           \n");
mprintf("    Version 1.01-b - April 2014      \n");
mprintf("http://www.controlsystemslab.com/rtsx\n");
mprintf("=====================================\n");

// Get root path of the toolbox
RTSX_ROOT = get_absolute_file_path( "startup_rtsx.sce" );
RTSX_DEMOS = RTSX_ROOT + "demos";
RTSX_TESTS = RTSX_ROOT + "tests";

// ============== common constants ================

pi = %pi;
false = 0;
true = 1;
False = %f;
True = %t;

// ============== all functions ===================

// Activate/deactivate external dependencies
load3rdPartyOn = %f;
// Load the functions
prot = funcprot(0);  
exec(RTSX_ROOT + "load_functions.sce", -1);
funcprot(prot);

// =======================================================================================






















