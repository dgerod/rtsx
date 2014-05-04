// =======================================================================================
// startup_rtsx.sce  
// www.controlsystemslab.com   Feb 2014
// ---------------------------------------------------------------------------------------
// Script that initialize the RTSX toolbox
// =======================================================================================

mprintf("Robotics Tools for Scilab/Xcos (RTSX) Version 1.01");
mprintf("by Control Systems Lab,  February 2014");
mprintf("http://www.controlsystemslab.com/rtsx");

// Get root path of the toolbox
RTSX_ROOT = get_absolute_file_path( "startup_rtsx.sce" );
RTSX_DEMOS = RTSX_ROOT + 'demos';

// ============== common constants ================

pi = %pi;
false = 0;
true = 1;
// boolean
False = (1==0);
True = (1==1);

// ============== all functions ===================

prot = funcprot(0);  
exec( RTSX_ROOT +"/load_functions.sce",-1 );
funcprot(prot);

// =======================================================================================






















