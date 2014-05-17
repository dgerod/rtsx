// =================================================================================
// InverseKinem.sci
// =================================================================================

function [Ret, Joints] = d2rInvKinem (KinemParams, TCP0)
//
// DESCRIPTION
//  Solves the Inverse Kinematics problem of a delta-2 robot. 
// PARAMETERS
//  KinemParams [IN] : Kinematics parameters [rf,lf,le,re].
//  TCP0        [IN] : Pose [x,y,z]' in RCS.
//  Joints     [OUT] : Position [j1,j2,j3]' in JCS (radians).
// RETURN
//  Ret : Success (1) or error (<0)
//
  
  Ret = 1;
  Joints = zeros(2,1);
 
  // Constants
  // --------------------------------------------
  
  s180 = 0;
  c180 = -1;        
  
  // Inputs
  // --------------------------------------------

  // Check if y is different than 0
  // This robot can only be moved in (x,z)
  if TCP0(2) <> 0.0 then 
    Ret = -1;
    return [Ret,Joints];
  end

  x0 = TCP0(1);
  z0 = TCP0(3);
  
  // Solve IK of each leg
  // --------------------------------------------
  
  theta1 = 0;
  theta2 = 0;
  
  // Leg 1 [ F1,G1,E1 ] ---
  // It is right leg.
  
  x = x0;
  z = z0;
  
  [status,theta1] = __d2rCalcJointAngle( KinemParams,[x,0,z] );
  if status <> 1 then
    Ret = -1;
    return [Ret,Joints];
  end
  
  // Leg 2 [ F2,G2,E2 ] ---
  // It is left leg.
  // Rotate coords to +180 degrees
  
  x = x0*c180 - z0*s180;
  z = z0;
  
  [status,theta2] = __d2rCalcJointAngle( KinemParams,[x,0,z] );  
  if status <> 1 then
    Ret = -2;
    return [Ret,Joints];
  end
  
  // Return joints
  // --------------------------------------------
  
  Ret = 1;
  Joints = [theta1,theta2]';
  return [Ret,Joints];
  
endfunction

// ---------------------------------------------------------------------------------

function [Ret, Theta] = __d2rCalcJointAngle (KinemParams, TCP0)
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
  
  tcp_x = TCP0(1);
  tcp_z = TCP0(3);
  
  if tcp_z == 0 then
    // ERROR - This is necessary to avoid division by zero.
    Ret = -1;
    return [Ret,Theta]; 
  end
  
  // Calcualte point Fi(x,y,z)
  xF = rf;
  zF = 0;
  
  // Calculate Ei coordinates using P position.
  xE = tcp_x + re;    
  zE = tcp_z;
  
  // Calculate Gi point 
  // It is the intersection of circles (Fi,lf) and (Ei,le)  
  // --------------------------------------------
  
  // Parameters of the circles: circle 0 (x0,y0r0) and circle 1 (x1,y1,r1)
  x0 = xF;  y0 = zF;  r0 = lf;
  x1 = xE;  y1 = zE;  r1 = le;
  
  // dx and dy are the vertical and horizontal distances between
  // the circle centers, they are the components of d.
  dx = x1 - x0;
  dy = y1 - y0;
  
  // Determine the straight-line distance between the centers. 
  d = sqrt( (dy*dy) + (dx*dx) );
  
  // Check for solvability
  if d > (r0 + r1)
    // ERROR - No solution. Circles do not intersect.
    Ret = -2;
    return [Ret,Theta];         
  end 

  if d < abs(r0 - r1)    
    // ERROR - No solution. One circle is contained in the other 
    Ret = -3;
    return [Ret,Theta];         
  end

  // 'point 2' is the point where the line through the circle
  // intersection points crosses the line between the circle
  // centers.  
  
  // Determine the distance from point 0 to point 2. 
  a = ((r0*r0) - (r1*r1) + (d*d)) / (2.0*d) ;
  
  // Determine the coordinates of point 2. 
  x2 = x0 + (dx * a/d);
  y2 = y0 + (dy * a/d);
  
  // Determine the distance from point 2 to either of the
  // intersection points.
  h = sqrt((r0*r0) - (a*a));
  
  // Now determine the offsets of the intersection points from
  // point 2.
  // Determine the absolute intersection points. 
  
  // Select P1 or P2
  // We must chose smaller point
  xi_1 = x2 - dy*(h/d);
  yi_1 = y2 + dx*(h/d);
  
  xG = xi_1;   zG = yi_1;

  // Caculate theta using Fi and Gi points
  angle = atan(-zG,(xG - xF))

  // --------------------------------------------

  Ret = 1;
  Theta = angle;
  return [Ret,Theta];         
  
endfunction

// =================================================================================
 