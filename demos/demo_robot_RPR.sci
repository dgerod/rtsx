// =============================================================================
// demo_robot_RPR.sci
// www.controlsystemslab.com  August 2012
// Demostration of RPR robot
// =============================================================================

mprintf("=========== RPR demonstration - Start =======\n");

mprintf("Creating robot model and joint sequence...\n");
exec(RTSX_ROOT + 'models/ex1_mdl.sce',-1);
exec(RTSX_ROOT + 'models/ex1_genqs2.sce',-1);

figure("figure_name","Robot model");
PlotRobot(ex1_robot,q0,'figure',get(gcf(),"figure_id"),'hold');
figure("figure_name","Robot frames");
PlotRobotFrame(ex1_robot,q0,'figure',get(gcf(),"figure_id"),'hold');

figure("figure_name","Movements");
AnimateRobot(ex1_robot,qs,'figure',get(gcf(),"figure_id"),'hold');

mprintf("=========== RPR demonstration - End =======\n");

// =============================================================================
