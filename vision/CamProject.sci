// CamProject.sci  
// www.controlsystemslab.com  April 2013
//CentralCamera.project Project world points to image plane
//
// UV = CamProject(cam,P, OPTIONS) are the image plane coordinates (2xN) corresponding
// to the world points P (3xN).
//
// - If Tcam (4x4xS) is a transform sequence then UV (2xNxS) represents the sequence 
//   of projected points as the camera moves in the world.
// - If Tobj (4x4xS) is a transform sequence then UV (2xNxS) represents the sequence 
//   of projected points as the object moves in the world.
//
// L = C.project(L, OPTIONS) are the image plane homogeneous lines (3xN) corresponding
// to the world lines represented by a vector of Plucker coordinates (1xN).
//
// Options::
// 'Tobj',T   Transform all points by the homogeneous transformation T before
//            projecting them to the camera image plane.
// 'Tcam',T   Set the camera pose to the homogeneous transformation T before
//            projecting points to the camera image plane.  Temporarily overrides 
//            the current camera pose C.T.
//
// Notes::
// - Currently a camera or object pose sequence is not supported for
//   the case of line projection.

function uv = CamProject(cam, P, varargin)
    uv = _Cam_Project(cam, P, varargin);
endfunction

function uv = camproject(cam, P, varargin)
    uv = _Cam_Project(cam, P, varargin);
endfunction

function uv = _Cam_Project(cam, P, varargin)
    varargin = varargin($);
    varnum=length(varargin);

    opt.Tobj = [];
    opt.Tcam = [];

    for iv =1:varnum 
         if type(varargin(iv))==10 then  // check if string 
              varargin(iv)=convstr(varargin(iv),'l'); // convert to lower 
          end

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
           end // select varargin(iv)
      end // for iv=1:2:varnum-1
    
    np = numcols(P);
        
    if isempty(opt.Tcam)
        opt.Tcam = cam.T;
    end

    if ndims(opt.Tobj) == 3 & ndims(opt.Tcam) == 3
        error('cannot animate object and camera simultaneously');
    end

    // project points
    if ndims(opt.Tobj) == 3
        // animate object motion, static camera
        
        // get camera matrix for this camera pose
        C = cam.C*opt.Tcam;
        
        // make the world points homogeneous
        if numrows(P) == 3
           P = e2h(P);
        end
        
        for iframe=1:size(opt.Tobj,3)
            
            // transform all the points to camera frame
            X = C * opt.Tobj(:,:,iframe) * P;     // project them
            
            //X(3,X(3,:)<0) = NaN;    // points behind the camera are set to NaN
            X = h2e(X);            // convert to Euclidean coordinates
            
            //if cam.noise
                // add Gaussian noise with specified standard deviation
            //    X = X + diag(cam.noise) * randn(size(X));
            //end
            uv(:,:,iframe) = X;
        end
    else
        // animate camera, static object
        
        // transform the object
        if ~isempty(opt.Tobj)
            P = homtrans(opt.Tobj, P);
        end
        
        // make the world points homogeneous
        if numrows(P) == 3
            P = e2h(P);
        end
        //pause;
        for iframe=1:size(opt.Tcam,3)
            C = cam.C*inv(opt.Tcam(:,:,iframe));
            
            // transform all the points to camera frame
            X = C * P;              // project them
            //X(3,X(3,:)<0) = NaN;    // points behind the camera are set to NaN
            X = h2e(X);            // convert to Euclidean coordinates
            
            //if c.noise
                // add Gaussian noise with specified standard deviation
               // X = X + diag(c.noise) * randn(size(X));
            //end
            uv(:,:,iframe) = X;
        end
    end



endfunction
