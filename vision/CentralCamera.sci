// CentralCamera.sci 
// www.controlsystemslab.com  April 2013
// implements CentralCamera command partially compatible with
// rvc tools 

function cam=CentralCamera(varargin)
    cam=_Central_Camera(varargin);
    
endfunction

function cam=centralcamera(varargin)
    cam=_Central_Camera(varargin);
endfunction

//
//   % C = CentralCamera() creates a central projection camera with canonic
//   % parameters: f=1 and name='canonic'.
//   %
//   % C = CentralCamera(OPTIONS) as above but with specified parameters.
//   %
//   % Options::
//   % 'name',N                  Name of camera
//   % 'focal',F                 Focal length [metres]
//   % 'distortion',D            Distortion vector [k1 k2 k3 p1 p2]
//   % 'distortion-bouguet',D    Distortion vector [k1 k2 p1 p2 k3]
//   % 'default'                 Default camera parameters: 1024x1024, f=8mm,
//   %                           10um pixels, camera at origin, optical axis
//   %                           is z-axis, u- and v-axes parallel to x- and 
//   %                           y-axes respectively.
//   % 'image',IM                Display an image rather than points
//   % 'resolution',N            Image plane resolution: NxN or N=[W H]
//   % 'sensor',S                Image sensor size in metres (2x1)
//   % 'centre',P                Principal point (2x1)
//   % 'pixel',S                 Pixel size: SxS or S=[W H]
//   % 'noise',SIGMA             Standard deviation of additive Gaussian 
//   %                           noise added to returned image projections
//   % 'pose',T                  Pose of the camera as a homogeneous 
//   %                           transformation
//   % 'color',C                 Color of image plane background (default [1 1 0.8])
//   %
//


function cam=_Central_Camera(varargin)
    varargin = varargin($);
    varnum=length(varargin);

     // initialize cam data structure
     cam.name = '';
     
     cam.comment = '';
 
     cam.T = eye(4,4);
     // default values
    cam.f = 8e-3;   // focal length
    cam.rho = [1,1]; 
    cam.npix = [];  // 1Mpix image plane
    cam.pp = [0,0];    // principal point in the middle
    cam.limits = [-1 1 -1 1];
    cam.perspective = True;
    cam.noise = 0;
    if varnum==0 then
        cam.name = 'canonic';
        cam.distortion = [];
        return;
    end

    for iv =1:varnum  
         if type(varargin(iv))==10 then  // check if string 
              varargin(iv)=convstr(varargin(iv),'l'); // convert to lower 
          end
          if varargin(iv)=='centre' then  // for the brits
              varargin(iv)='center';  
          end
          if type(varargin(iv))==10 then
          
              select varargin(iv),
                case 'default' then
                      cam.f = 8e-3;   // focal length
                      cam.rho = [10e-6, 10e-6]; // square pixels 10um side
                      cam.npix = [1024,1024];  // 1Mpix image plane
                      cam.pp = [512, 512];    // principal point in the middle
                       cam.limits = [0 1024 0 1024];
                      cam.perspective = True;
                       cam.noise = 0;                 
    
                  case 'name' then 
                      if  type(varargin(iv+1))==10 then
                           cam.name = varargin(iv+1);
                      else error("Camera name must be a string");
                      end
                  case 'comment' then
                      if  type(varargin(iv+1))==10 then
                          cam.comment = varargin(iv+1);
                      else error("Comment must be a string");
                      end 
                   case 'focal' then
                     if type(varargin(iv+1))==1 & size(varargin(iv+1))==[1,1]
                         cam.f = varargin(iv+1);
                     else error('Wrong data type for focal length');
                     end                             
                   case 'pixel' then
                     if type(varargin(iv+1))==1 & size(varargin(iv+1))==[1,1]
                         cam.rho = varargin(iv+1)*[1,1];
                     elseif  type(varargin(iv+1))==1 & size(varargin(iv+1))==[1,2]   
                         cam.rho = varargin(iv+1);
                     else error('Wrong data type for pixel size');
                     end                             
                   case 'resolution' then
                     if type(varargin(iv+1))==1 & size(varargin(iv+1))==[1,2]
                         cam.npix = varargin(iv+1);
                     else error('Wrong data type for resolution. Expect [nw, nh]');
                     end   
                   case 'center' then
                     if type(varargin(iv+1))==1 & size(varargin(iv+1))==[1,2]
                         cam.pp = varargin(iv+1);
                     else error('Wrong data type for principal point. Expect [ou, ov]');
                     end   
                   case 'pose' then
                     if ishomog(varargin(iv+1))
                         cam.T = varargin(iv+1);
                     else error('Wrong data type for pose. Must be a 4 x 4 homogeneous matrix');
                     end
                   case 'noise' then
                     if type(varargin(iv+1))==1 & size(varargin(iv+1))==[1,1]
                         cam.noise = varargin(iv+1);
                     else error('Wrong data type for camera noise');
                     end                      
                     
               end // select varargin(iv)
          end // if type(varargin(iv)==10)
      end // for iv=1:varnum-1
      cam.u0 = cam.pp(1);
      cam.v0 = cam.pp(2);
      // compute camera parameters
      cam.C = [cam.f/cam.rho(1), 0, cam.pp(1), 0;
                0, cam.f/cam.rho(2), cam.pp(2) 0;
                0, 0, 1,0]*inv(cam.T);
      cam.K = [   cam.f/cam.rho(1)   0           cam.pp(1) ;
                    0          cam.f/cam.rho(2)    cam.pp(2);
                    0          0           1];
                
endfunction

// --------------- UpdateCamera ------------------------------
// use this function to adjust camera parameters and recompute C and K matrices

function cam=UpdateCamera(cam, varargin)
    cam=_Update_Camera(cam, varargin);
    
endfunction

function cam=updatecamera(cam, varargin)
    cam=_Update_Camera(cam, varargin);
endfunction

function cam=_Update_Camera(cam, varargin)
    varargin = varargin($);
    varnum=length(varargin);


    for iv =1:varnum  // select only command at odd position
         if type(varargin(iv))==10 then  // check if string 
              varargin(iv)=convstr(varargin(iv),'l'); // convert to lower 
          end
          if varargin(iv)=='centre' then  // for the brits
              varargin(iv)='center';  
          end
          if type(varargin(iv))==10 then
          
              select varargin(iv),
                case 'default' then
                      cam.f = 8e-3;   // focal length
                      cam.rho = [10e-6, 10e-6]; // square pixels 10um side
                      cam.npix = [1024,1024];  // 1Mpix image plane
                      cam.pp = [512, 512];    // principal point in the middle
                       cam.limits = [0 1024 0 1024];
                      cam.perspective = True;
                       cam.noise = 0;                 
    
                  case 'name' then 
                      if  type(varargin(iv+1))==10 then
                           cam.name = varargin(iv+1);
                      else error("Camera name must be a string");
                      end
                  case 'comment' then
                      if  type(varargin(iv+1))==10 then
                          cam.comment = varargin(iv+1);
                      else error("Comment must be a string");
                      end 
                   case 'focal' then
                     if type(varargin(iv+1))==1 & size(varargin(iv+1)==[1,1])
                         cam.f = varargin(iv+1);
                     else error('Wrong data type for focal length');
                     end                             
                   case 'pixel' then
                     if type(varargin(iv+1))==1 & size(varargin(iv+1)==[1,1])
                         cam.rho = varargin(iv+1)*[1,1];
                     elseif  type(varargin(iv+1))==1 & size(varargin(iv+1)==[1,2])   
                         cam.rho = varargin(iv+1);
                     else error('Wrong data type for pixel size');
                     end                             
                   case 'resolution' then
                     if type(varargin(iv+1))==1 & size(varargin(iv+1)==[1,2])
                         cam.npix = varargin(iv+1);
                     else error('Wrong data type for resolution. Expect [nw, nh]');
                     end   
                   case 'center' then
                     if type(varargin(iv+1))==1 & size(varargin(iv+1)==[1,2])
                         cam.pp = varargin(iv+1);
                     else error('Wrong data type for principal point. Expect [ou, ov]');
                     end   
                   case 'pose' then
                     if ishomog(varargin(iv+1))
                         cam.T = varargin(iv+1);
                     else error('Wrong data type for pose. Must be a 4 x 4 homogeneous matrix');
                     end
                   case 'noise' then
                     if type(varargin(iv+1))==1 & size(varargin(iv+1)==[1,1])
                         cam.noise = varargin(iv+1);
                     else error('Wrong data type for camera noise');
                     end                      
                     
               end // select varargin(iv)
          end // if type(varargin(iv)==10)
      end // for iv=1:varnum-1
      cam.u0 = cam.pp(1);
      cam.v0 = cam.pp(2);
      // compute camera parameters
      cam.C = [cam.f/cam.rho(1), 0, cam.pp(1), 0;
                0, cam.f/cam.rho(2), cam.pp(2) 0;
                0, 0, 1,0]*inv(cam.T);
      cam.K = [   cam.f/cam.rho(1)   0           cam.pp(1) ;
                    0          cam.f/cam.rho(2)    cam.pp(2);
                    0          0           1];
                
endfunction





