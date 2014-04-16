// rprdemo.sci  building an RPR robot using RTSX
// www.controlsystemslab.com  August 2012

function rprdemo()

printf("\nRPR Robot Demonstration\n"); 

printf("\nCreating robot model and joint variable sequence...\n");
exec('./models/mdl_ex1.sce',-1);
exec('./models/ex1_genqs2.sce',-1);
printf("\nSee PlotRobot( ) in window number 1\n");
PlotRobot(ex1_robot,q0,'figure',1);
printf("\nSee PlotRobotFrame( ) in window number 2\n");
PlotRobotFrame(ex1_robot,q0,'figure',2);
printf("\nSee AnimateRobot( ) in window number 3\n");
AnimateRobot(ex1_robot,qs,'figure',3);

printf("\n=========== End of RPR Robot Demonstration =======\n");

   
endfunction