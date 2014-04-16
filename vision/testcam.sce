// TESTCAM.SCE   
// Test camera functions

cam = CentralCamera('focal', 0.015, 'pixel', 10e-6, ...
'resolution', [1280 1024], 'centre', [640 512], 'name', 'my camera');
P = mkgrid(3, 0.2, 'T', transl([0,0,1.0]));
//Tcam = transl([-1,0,0.5])*troty(0.9);
//uv = camproject(cam,P);
//camplot(cam,P,'Tcam', Tcam);

Tcam = transl([0,0,0.5])*troty(0);
//camplot(cam,P,'Tcam',Tcam,'style','o','color','green');
for idx = 0:0.1:1
    Tcamf = transl([0,0,0.5])*troty(0);
    camplot(cam,P,'Tcam',Tcamf,'style','o','color','green','figure',1);
    Tcam = transl([-idx,0,0.5])*troty(idx);
    if idx<1 then
        camplot(cam,P,'Tcam',Tcam,'style','+','color','red','figure',1);
    else
        camplot(cam,P,'Tcam',Tcam,'style','*','color','blue','figure',1);
    end
    //pause;
end
