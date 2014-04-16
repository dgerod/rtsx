// testframe.sce  building a kinematic chain for testing 
// www.controlsystemslab.com  August 2012
clear F1;
F1(1)=Frame(eye(4,4),'name','{U}');
F1(2)=Frame(transl([0 1 1]),'name','{V}');
F1(3)=Frame([],'name','{X}');
F1(4)=Frame(trotx(pi/2)*transl([1 1 0]),'rel','name','{W}');
F1(5)=Frame(trotz(pi/4)*transl([1 0 1]),'name','{Y}');
F1(6)=Frame(transl([1 1 1]),'rel','name','{T}');

// translation only
clear F2;
F2(1)=Frame(eye(4,4),'name','{U}');
F2(2)=Frame(transl([0 1 1]),'name','{V}');
F2(3)=Frame(transl([1 1 1]),'name','{W}');

// rotation only
clear F3;
F3(1)=Frame(eye(4,4),'name','{U}');
F3(2)=Frame(trotz(pi/2),'name','{V}');
F3(3)=Frame(trotx(pi/2),'name','{W}');

// ---- test PlotResolveFrame function
clear Fa;
clear kc_a;
clear Fb;
clear kc_b;
Fa(1)=Frame(eye(4,4),'name','U');
Fa(2)=Frame(trotz(pi/4)*transl([1 1 1]),'abs','name','V');
Fa(3)=Frame([],'name','X');
Fa(4)=Frame(transl([1 2 1]),'name','W');

kc_a = SerialFrame(Fa,'name','Chain a');

Fb(1)=Frame(eye(4,4),'name','U');
Fb(2)=Frame(trotz(pi/4)*transl([-1 -1 -1]),'abs','name','A');

Fb(3)=Frame(trotx(pi/4)*transl([-1 2 1]),'name','B');

kc_b = SerialFrame(Fb,'name','Chain b');