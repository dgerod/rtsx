// =================================================================================
// DirectKinem.sci
// =================================================================================

function [Ret, TCP0] = d3rDirKinem (KinemParams, Joints)
//
// DESCRIPTION
//  Solves the Direct Kinematics problem of a delta-3 robot. 
// PARAMETERS
//  KinemParams [IN] : Kinematics parameters [rf,lf,le,re].
//  Joints      [IN] : Position [j1,j2,j3]' in JCS (radians).
//  TCP0       [OUT] : Pose [x,y,z]' in RCS.
// RETURN
//  Ret : Success (1) or error (<0)
//

  Ret = 1;
  TCP0 = zeros(3,1);
  
  // Constants
  // --------------------------------------------
  
  sq3 = sqrt(3.0);
  pi = 3.141592653;   
  s120 = sq3/2.0;   
  c120 = -0.5;        
  t60 = sq3;
  s30 = 0.5;
  t30 = 1/sq3;
  c30 = s30/t30;
  
  // Inputs
  // --------------------------------------------
  
  rf = KinemParams(1);
  lf = KinemParams(2);
  le = KinemParams(3);
  re = KinemParams(4);
  
  theta1 = Joints(1);
  theta2 = Joints(2);
  theta3 = Joints(3);
  
  // Solve kinematics
  // --------------------------------------------
  
  ac = rf - re;
  
  // 1. Calcualte G1', G2' and G3'
  // ///////////////////////////////
  
  // 1.1. G1'
  
  d1 = ac + lf*cos(theta1);
  
  x1 = 0;
  y1 = -d1;
  z1 = -lf*sin(theta1);
   
  G1p = [x1 y1 z1];
  
  // 1.2. G2' 
  
  d2 = ac + lf*cos(theta2);
  
  x2 = d2*c30;
  y2 = d2*s30;
  z2 = -lf*sin(theta2);
  
  G2p = [x2 y2 z2];
  
  // 1.3. G3'
  
  d3 = ac + lf*cos(theta3);
  
  x3 = -(d3)*c30
  y3 = (d3)*s30;
  z3 = -lf*sin(theta3);
  
  G3p = [x3 y3 z3];  
  
  // 2. TCP-0 as intersection of 3 spheres
  // ///////////////////////////////
  
  // 2.1. Calculate z0
  
  dnm = (y2-y1)*x3 - (y3-y1)*x2;
  
  w1 = y1^2 + z1^2;
  w2 = x2^2 + y2^2 + z2^2;
  w3 = x3^2 + y3^2 + z3^2;
  
  // x = (a1*z + b1)/dnm
  a1 = (z2-z1)*(y3-y1) - (z3-z1)*(y2-y1);
  b1 = -((w2-w1)*(y3-y1) - (w3-w1)*(y2-y1))/2.0;

  // y = (a2*z + b2)/dnm;
  a2 = -(z2-z1)*x3 + (z3-z1)*x2;
  b2 = ((w2-w1)*x3 - (w3-w1)*x2)/2.0;  

  //    2      
  // A*z  + B*z + C = 0
  A = a1^2 + a2^2 + dnm*dnm;
  B = 2*(a1*b1 + a2*(b2-y1*dnm) - z1*dnm*dnm);
  C = (b2-y1*dnm)*(b2-y1*dnm) + b1*b1 + dnm*dnm*(z1*z1 - le*le);  
  
  // discriminant
  //        2
  // D =  B  - 4AC    
  
  D = B*B - 4.0*A*C;  
 
  if D < 0 | A == 0.0 then
    
    // ERROR - Non-existing point
    Ret = -1;
    TCP0 = [ 0,0,0 ]';
    return [Ret,TCP0]; 	 
    
  end

  z0 = -0.5*(B+sqrt(D))/A;
  
  // 2.2. Calculate x0 and y0
  
  x0 = (a1*z0 + b1)/dnm;
  y0 = (a2*z0 + b2)/dnm;
  
  // Return TCP-0
  // --------------------------------------------

  TCP0 = [x0,y0,z0]';
  
  Ret = 1;	
  return [Ret,TCP0]; 
  
endfunction 

// =================================================================================

