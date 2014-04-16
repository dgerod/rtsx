// CamPlot.sci  
//   %Camera.plot Plot points on image plane
//  %
//  % C.plot(P, OPTIONS) projects world points P (3xN) to the image plane and plots them.  If P is 2xN
//  % the points are assumed to be image plane coordinates and are plotted directly.
//  %
//  % UV = C.plot(P) as above but returns the image plane coordinates UV (2xN).
//  %
//  % - If P has 3 dimensions (3xNxS) then it is considered a sequence of point sets and is
//  %   displayed as an animation.
//  %
//  % C.plot(L, OPTIONS) projects the world lines represented by the
//  % array of Plucker objects (1xN) to the image plane and plots them.
//  %
//  % LI = C.plot(L, OPTIONS) as above but returns an array (3xN) of
//  % image plane lines in homogeneous form.
//  %
//  % Options::
//  % 'Tobj',T         Transform all points by the homogeneous transformation T before
//  %                  projecting them to the camera image plane.
//  % 'Tcam',T         Set the camera pose to the homogeneous transformation T before
//  %                  projecting points to the camera image plane.  Overrides the current
//  %                  camera pose C.T.
//  % 'fps',N          Number of frames per second for point sequence display
//  % 'sequence'       Annotate the points with their index
//  % 'textcolor',C    Text color for annotation (default black)
//  % 'textsize',S     Text size for annotation (default 12)
//  % 'drawnow'        Execute MATLAB drawnow function
//  %
//  % Additional options are considered MATLAB linestyle parameters and are passed 
//  % directly to plot.

function uv = CamPlot(cam, P, varargin)
    uv = _Cam_Plot(cam, P, varargin);
endfunction

function uv = camplot(cam, P, varargin)
    uv = _Cam_Plot(cam, P, varargin);
endfunction

function uv = _Cam_Plot(cam, P, varargin)
    varargin = varargin($);
    varnum=length(varargin);
    
    fnum = max(winsid())+1;
    setfignum = False;
    opt.Tobj = eye(4,4);
    opt.Tcam = eye(4,4);
    opt.fps = 5;
    opt.sequence = False;
    opt.textcolor = 'k';
    opt.textsize = 12;
    opt.drawnow = False;
    opt.grid = False;
    opt.hold = False;
    opt.pcolor = 1;
    opt.pstyle = '*';  
    opt.psize = 3;  // mark size
    for iv =1:varnum  
         if type(varargin(iv))==10 then  // check if string 
              varargin(iv)=convstr(varargin(iv),'l'); // convert to lower 
          end
          if type (varargin(iv))==10 then
          
              select varargin(iv),
                   case 'tobj' then
                     if ishomog(varargin(iv+1))
                         opt.Tobj = varargin(iv+1);
                     else error('Wrong data type for Tobj. Must be a 4 x 4 homogeneous matrix');
                     end
    
                   case 'tcam' then
                     if ishomog(varargin(iv+1))
                         opt.Tcam = varargin(iv+1);
                     else error('Wrong data type for Tcam. Must be a 4 x 4 homogeneous matrix');
                     end
                   case 'figure' then  // can put figure number as arg
                        fnum = varargin(iv+1); 
                        setfignum = True;  
                    case 'hold' then
                       
                         opt.hold = True; 
                         if ~setfignum then  // hold last window
                                 curwin = winsid();
                                if curwin~=[] fnum = curwin(length(curwin));
                                 end
                         end     
                   case 'color' then
                         opt.pcolor = colormap(varargin(iv+1));
                   case 'style' then
                         opt.pstyle = varargin(iv+1);

                             
                   case 'size' then
                         opt.psize = varargin(iv+1);     
                         if opt.psize < 0 then
                             opt.psize = 0;
                         elseif opt.psize > 5 then
                             opt.psize = 5;
                         end                                             
                    case 'grid' then 
                       opt.grid = True;
                      
               end // select varargin(iv)
          end  // if type (varargin(iv))== 10     
      end // for iv=1:2:varnum-1    
     if find(winsid()==fnum) & ~opt.hold then           
        xdel(fnum);
     end    
    
     figure(fnum); 
     nr = numrows(P);
     if nr == 3 | nr ==4 // project 3D world using CamProject()
         uv = camproject(cam,P,'tcam',opt.Tcam);
     else
         uv = P;
     end   
     
     plot(uv(1,:),uv(2,:),opt.pstyle);
     he=gce();
     pts = he.children;
     pts.mark_foreground = opt.pcolor;
     pts.mark_size = opt.psize;
     xlabel('u (pixels)');
     ylabel('v (pixels)');
     title(cam.name);
     ha=gca();
     ha.axes_reverse = ["off","on","off"];
     ha.data_bounds = [0,0;cam.npix(1),cam.npix(2)];
     if opt.grid then
         xgrid;
     end
     
     
endfunction
