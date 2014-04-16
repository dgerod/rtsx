// hw32_genqs.sce  generates qs for robot in Homework 3 problem 1
// for testing purpose
// www.controlsystemslab.com   July 2012

t = [0:0.05:1]';        // Time data
tinv = flipdim(t,1);    // inverse of time

// section 1: move together
q1a=t*pi/4;
q2a=t*pi/4;
q3a=-t*pi/2;

// pause together
t_length=size(t,1);
p_length=10;   // pause between move
pt=ones(p_length,1);
q1p1=q1a(t_length)*pt;
q2p1=q2a(t_length)*pt;
q3p1=q3a(t_length)*pt;

// section 2: only joint 3 moves 
q3b=-tinv*pi/2;
q3b_length=size(q3b,1);
p3b=ones(q3b_length,1);
q2p2 = q2a(t_length)*p3b;  // joint 2 pauses
q1p2 = q1a(t_length)*p3b;  // joint 1 pauses

// pause together
q3p2 = 0*pt;
q2p3 = q2a(t_length)*pt;
q1p3 = q1a(t_length)*pt;

// section 3: only joint 2 moves
q2b=tinv*pi/4;
q2b_length=size(q2b,1);
p2b=ones(q2b_length,1);
q1p4 = q1a(t_length)*p2b;
q3p3 = 0*p2b;

// pause together
q3p4 = 0*pt;
q2p4 = 0*pt;
q1p5 = q1a(t_length)*pt;

// section 4: only joint 1 moves
q1b=tinv*pi/4;
q1b_length=size(q1b,1);
p1b=ones(q1b_length,1);
q2p5=0*p1b;
q3p5=0*p1b;


q1=[q1a; q1p1; q1p2; q1p3; q1p4; q1p5; q1b];
q2=[q2a; q2p1; q2p2; q2p3; q2b; q2p4; q2p5];
q3=[q3a; q3p1; q3b; q3p2; q3p3; q3p4; q3p5];

qs = [q1 q2 q3];

