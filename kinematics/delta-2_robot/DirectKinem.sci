// =================================================================================
// DirectKinem.sci
// =================================================================================

function [Ret, TCP0] = d2rDirKinem (KinemParams, Joints)
//
// DESCRIPTION
//  Solves the Direct Kinematics problem of a delta-2 robot. 
// PARAMETERS
//  KinemParams [IN] : Kinematics parameters [rf,lf,le,re].
//  Joints      [IN] : Position [j1,j2]' in JCS (radians).
//  TCP0       [OUT] : Pose [x,y,z]' in RCS.
// RETURN
//  Ret : Success (1) or error (<0)
//

  Ret = 1;
  TCP0 = zeros(3,1);
  
  // Inputs
  // --------------------------------------------
  
  rf = KinemParams(1);
  lf = KinemParams(2);
  le = KinemParams(3);
  re = KinemParams(4);
  
  theta1 = Joints(1);
  theta2 = Joints(2);
  
  // Solve kinematics
  // --------------------------------------------
  
  ac = rf - re;
  
  // 1. Calcualte G1' and G2'
  // ///////////////////////////////
  
  // 1.1. G1'
  
  d1   = ac + lf*cos(theta1);  
  xG1p = d1;
  zG1p = -lf*sin(theta1);
   
  G1p = [xG1p zG1p];
  
  // 1.2. G2' 
  
  d2   = ac + lf*cos(theta2);  
  xG2p = -d2;
  zG2p = -lf*sin(theta2);
  
  G2p = [xG2p zG2p];
   
  // 2. TCP-0 as intersection of 2 circles
  // ///////////////////////////////
  
  // Center and radius of the circles
  x0 = xG1p;  y0 = zG1p;  r0 = le;
  x1 = xG2p;  y1 = zG2p;  r1 = le;
  
  // dx and dy are the vertical and horizontal distances between
  // the circle centers, they are the components of d.
  dx = x1 - x0;
  dy = y1 - y0;
  
  // Determine the straight-line distance between the centers. 
  d = sqrt( (dy*dy) + (dx*dx) );

  // Check for solvability
  if d > (r0 + r1)

    // ERROR - No solution. circles do not intersect.
    Ret = -1;
    TCP0 = [ 0,0,0 ]';
    return [Ret,TCP0]; 
  end 

  if d < abs(r0 - r1)    
    // ERROR - No solution. one circle is contained in the other 
    Ret = -2;
    TCP0 = [ 0,0,0 ]';
    return [Ret,TCP0]; 
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
  // We have to chose smaller point

  xi_1 = x2 - dy*(h/d);
  yi_1 = y2 + dx*(h/d);
  
  x0 = xi_1;
  z0 = yi_1;
  
  // Return TCP-0
  // --------------------------------------------

  TCP0 = [x0,0,z0]';  
  Ret = 1;	
  return [Ret,TCP0]; 
  
endfunction 

// =================================================================================

