// plotresolveframetest.sce  test PlotResolveFrame( )
// www.controlsystemslab.com   August 2012

// first create a sequence of frames
//clear F1;
//F1(1)=Frame(eye(4,4),'name','U');  
//F1(3)=Frame(transl([1 1 0.5])*trotx(pi/2),'name','V'); 
//F1(2)=Frame([],'name','X');  // missing frame at location 4
//F1(4)=Frame(transl([1 1 1])*angvec2t(pi/2, [1 1 1]),'name','W');  
//fc1=SerialFrame(F1,'name','Chain 1'); // incomplete chain to be solved
//clear F2;
//F2(1)=Frame(eye(4,4),'name','U');
//F2(2)=Frame(transl([-3 -2 -1])*eul2t([pi/6 pi/3 pi/4]),'name','A');
//F2(3)=Frame(transl([-3 -4 -1])*troty(pi/4),'name','W');
//fc2=SerialFrame(F2,'name','Chain 2');  // chain 2 is completed
//[Trel,Tabs,fc1r]=PlotResolveFrame(fc1,fc2);

clear F;

T10 = [1 0 0 0; 0 1 0 1; 0 0 1 1;0 0 0 1];

T20 = [1 0 0 -0.5;0 1 0 1.5;0 0 1 1.1; 0 0 0 1];

T30 = [0 1 0 -0.5;1 0 0 1.5;0 0 -1 3;0 0 0 1];


F(1) = Frame(eye(4,4),'name','0');
F(2) = Frame(T10,'name','1');
F(3) = Frame(T20,'abs','name','2');
F(4) = Frame([],'name','3');  // blank frame to find

fc1 = SerialFrame(F,'name','Chain 1');

T32 = PlotResolveFrame(fc1,T30);