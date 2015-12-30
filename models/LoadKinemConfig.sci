// =================================================================================
// LoadKinemConfig.sci
// =================================================================================

function [Ret,KinemParams] = d3rLoadKinemConfig (Model)
//
// DESCRIPTION
//  Load configuration (rf,lf,le,re) parameters of a delta-3 robot
//  The model type is specificated using 'Model' input which is a string. If the model 
//  does not exist the function returns an error.
//  
// PARAMETERS
//  Model        [IN] : Model name (string).
//  KinemParams [OUT] : Kinematics parameters (rf,lf,le,re) [1x4].
// RETURN
//  Ret : Success (1) or error (<0)
//

  Ret = 1;
  
  select Model
    
  // Trossen Robotics web
  // http://forums.trossenrobotics.com/tutorials/introduction-129/delta-robot-kinematics-3276/
  case "LegoDelta" then
    f  = 132.01114;
    lf = 112.0;
    le = 232.0;
    e  = 33.19764;
    
  // MicroDELTA-240
  // Page 46 in DSc. report of R.Clavel, 1991
  case "MicroDELTA-240"
    // f = RA*(2/tan(30)) = RA*(2*sqrt(3))
    rf = 67;
    lf = 80.0;
    le = 160.0;
    // e = RB*(2/tan(30)) = RB*(2*sqrt(3))
    re = 17;
    
  // Unknown model      
  else
    errMsg = sprintf( "Unknown robot model: %s",Model );
    
    Ret = -1;
    error( errMsg ); // By defaut error stops execution and resume to prompt level.
  end
  
  KinemParams(1) = rf;
  KinemParams(2) = lf;
  KinemParams(3) = le;
  KinemParams(4) = re;

  return [Ret,KinemParams];
  
endfunction 

// =================================================================================
