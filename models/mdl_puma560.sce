// mdl_puma560.sce    make Puma 560 model
// www.controlsystemslab.com   July 2012

clear L
 //            th    d       a         alpha
L(1) = Link([ 0     0       0         %pi/2    0]);
L(2) = Link([ 0 	0       0.4318	  0       0]);
L(3) = Link([ 0     0.15005	0.0203    -%pi/2   0]);
L(4) = Link([ 0     0.4318	0         %pi/2    0]);
L(5) = Link([ 0     0       0         -%pi/2   0]);
L(6) = Link([ 0     0 	    0          0      0]);

L(1).m = 0;
L(2).m = 17.4;
L(3).m = 4.8;
L(4).m = 0.82;
L(5).m = 0.34;
L(6).m = .09;

L(1).r = [ 0    0	   0 ];
L(2).r = [ -.3638  .006    .2275];
L(3).r = [ -.0203  -.0141  .070];
L(4).r = [ 0    .019    0];
L(5).r = [ 0    0	   0];
L(6).r = [ 0    0	   .032];

//L(1).I = [  0	 0.35	 0	 0	 0	 0];
//L(2).I = [  .13	 .524	 .539	 0	 0	 0];
//L(3).I = [   .066  .086	 .0125   0	 0	 0];
//L(4).I = [  1.8e-3  1.3e-3  1.8e-3  0	 0	 0];
//L(5).I = [  .3e-3   .4e-3   .3e-3   0	 0	 0];
//L(6).I = [  .15e-3  .15e-3  .04e-3  0	 0	 0];

L(1).Jm =  200e-6;
L(2).Jm =  200e-6;
L(3).Jm =  200e-6;
L(4).Jm =  33e-6;
L(5).Jm =  33e-6;
L(6).Jm =  33e-6;

L(1).G =  -62.6111;
L(2).G =  107.815;
L(3).G =  -53.7063;
L(4).G =  76.0364;
L(5).G =  71.923;
L(6).G =  76.686;

// viscous friction (motor referenced)
L(1).B =   1.48e-3;
L(2).B =   .817e-3;
L(3).B =    1.38e-3;
L(4).B =   71.2e-6;
L(5).B =   82.6e-6;
L(6).B =   36.7e-6;



// Coulomb friction (motor referenced)
L(1).Tc = [ .395	-.435];
L(2).Tc = [ .126	-.071];
L(3).Tc = [ .132	-.105];
L(4).Tc = [ 11.2e-3 -16.9e-3];
L(5).Tc = [ 9.26e-3 -14.5e-3];
L(6).Tc = [ 3.96e-3 -10.5e-3];

p560 = SerialLink(L,'name','Puma 560','manuf','Unimation','comment', 'viscous friction; params of 8/95'); 


p560 = UpdateRobotLink(p560,1,'I',[  0	 0.35	 0	 0	 0	 0]);
p560 = UpdateRobotLink(p560,2,'I',[  .13	 .524	 .539	 0	 0	 0]);
p560 = UpdateRobotLink(p560,3,'I',[   .066  .086	 .0125   0	 0	 0]);
p560 = UpdateRobotLink(p560,4,'I',[  1.8e-3  1.3e-3  1.8e-3  0	 0	 0]);
p560 = UpdateRobotLink(p560,5,'I',[  .3e-3   .4e-3   .3e-3   0	 0	 0]);
p560 = UpdateRobotLink(p560,6,'I',[   .15e-3  .15e-3  .04e-3  0	 0	 0]);
// some useful poses
//

q_z = [0 0 0 0 0 0]; // zero angles, L shaped pose
q_r = [0 %pi/2 -%pi/2 0 0 0]; // ready pose, arm up
q_s = [0 0 -%pi/2 0 0 0];
q_n=[0 %pi/4 %pi 0 %pi/4  0];


clear L