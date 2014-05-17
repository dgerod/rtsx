// =================================================================================
// InverseKinem.sci
// =================================================================================

function [Ret, Joints] = d3rInvKinem (KinemParams, TCP0)
//
// DESCRIPTION
//  Solves the Inverse Kinematics problem of a delta-3 robot. 
// PARAMETERS
//  KinemParams [IN] : Kinematics parameters [rf,lf,le,re].
//  TCP0        [IN] : Pose [x,y,z]' in RCS.
//  Joints     [OUT] : Position [j1,j2,j3]' in JCS (radians).
// RETURN
//  Ret : Success (1) or error (<0)
//

  Ret = 1;
  Joints = zeros(3,1);
  
  // Constants
  // --------------------------------------------
  
  sq3 = sqrt(3.0);
  s120 = sq3/2.0;   
  c120 = -0.5;        
  s240 = -s120
  c240 = c120;        
  
  // Inputs
  // --------------------------------------------
  
  x0 = TCP0(1);
  y0 = TCP0(2);
  z0 = TCP0(3);
  
  // Solve IK of each leg
  // --------------------------------------------
  
  theta1 = 0;
  theta2 = 0;
  theta3 = 0;
  
  // Leg 1 [ F1,G1,E1 ] ---
  // It is right leg.
  
  x = x0;
  y = y0;
  z = z0;
  
  [status,theta1] = __d3rCalcJointAngle( KinemParams,[x,y,z] );
  if status ~= 1
    Ret = -1;
    Joints = [theta1,theta2,theta3]';
    return [Ret,Joints];
  end
  
  // Leg 2 [ F2,G2,E2 ] --- 
  // Rotate coords to 240 (or -120) degrees
  
  x = x0*c240 - y0*s240;
  y = y0*c240 + x0*s240;
  z = z0;
 
  [status,theta2] = __d3rCalcJointAngle( KinemParams,[x,y,z] );  
  if status ~= 1
    Ret = -2;
    Joints = [theta1,theta2,theta3]';
    return [Ret,Joints];
  end
  
    // Leg 3 [ F2,G2,E2 ] ---
  // Rotate coords to +120 degrees
  
  x = x0*c120 - y0*s120;
  y = y0*c120 + x0*s120;
  z = z0;
  
  [status,theta3] = __d3rCalcJointAngle( KinemParams,[x,y,z] );  
  if status ~= 1
    Ret = -3;
    Joints = [theta1,theta2,theta3]';
    return [Ret,Joints];
  end
  
  // Return joints
  // --------------------------------------------
  
  Ret = 1;
  Joints = [theta1,theta2,theta3]';
  return [Ret,Joints];
  
endfunction

// ---------------------------------------------------------------------------------

function [Ret, Theta] = __d3rCalcJointAngle (KinemParams, TCP0)
//
// DESCRIPTION
//  Calculates the joint value that is defined as the angle on the YZ plane
//
  
  Ret = 1;
  Theta = 0;
  
  // Inputs
  // --------------------------------------------
  
  rf = KinemParams(1);
  lf = KinemParams(2);
  le = KinemParams(3);
  re = KinemParams(4);
  
  x0 = TCP0(1);
  y0 = TCP0(2);
  z0 = TCP0(3);
  
  if z0 == 0 then
    // ERROR - This is necessary to avoid division by zero.
    Ret = -1;
    return [Ret,Theta]; 
  end
  
  // Calcualte points Fi(x,y,z) and Gi(x,y,z)
  // --------------------------------------------
  
  y1 = -rf; 
  // shift center to edge
  y0 = y0 - re;    
  
  // z = a + b*y
  a = (x0*x0 + y0*y0 + z0*z0 +lf*lf - le*le - y1*y1)/(2*z0);
  b = (y1-y0)/z0;
  
  // discriminant
  d = -(a + b*y1)*(a+b*y1) + lf*(b*b*lf + lf);
  
  if d < 0 
    // ERROR - Non-existing point
    Ret = -2;
    return [Ret,Theta];         
  end 
  
  // choosing outer point
  yj = (y1 - a*b - sqrt(d))/(b*b + 1); 
  zj = a + b*yj;

  if yj > y1        
    k = %pi;                
  else
    k = 0.0;
  end
  
  // Caculate theta using Fi and Gi points
  angle = atan(-zj/(y1 - yj)) + k;

  // --------------------------------------------

  Ret = 1;
  Theta = angle;
  return [Ret,Theta];         
  
endfunction

// =================================================================================
 
