// =============================================================================
// scp_DirKinem.sci
// =============================================================================

function [Ret, TCP0] = scpDirKinem (TRobot, Joints)
//
// DESCRIPTION
//  Solves the Direct Kinematics problem of a Scara-PRR robot. 
// PARAMETERS
//  TRobot [IN] : Robot object [H,L1,L2]
//  Joints [IN] : Position [j1,j2,j3]' in JCS.
//  TCP0  [OUT] : Pose [x,y,z]' in RCS.
// RETURN
//  Ret : Success (1) or error (<0)
// DEPENDS ON
//  None
// SEE ALSO
//  scrInvKinem
//

  Ret = 1;
  TCP0 = zeros(1,3);
  
  // Prepare data ---
  
  h = TRobot(1); l1 = TRobot(2);  l2 = TRobot(3);
  J = Joints;

  // Solve Direct Kinematics problem ---
  
  // Solve RR joints
  [Ret,TCP0] = __scpDkRR( [l1,l2],J(2:3) );
 
  if Ret <> 1 then
     Ret = -1;
     Joints = zeros(2,3);      
     return [Ret,Joints];
   end
  
  // Solve prismatic joint 
  z = h + J(1);
  
  // Return the solutions  
  
  TCP0(3) = z;
  
  Ret = 1;
  return [Ret,TCP0];

endfunction

// -----------------------------------------------------------------------------

function [Ret, TCP0] = __scpDkRR (TRobot, Joints)

  Ret = 1;
  TCP0 = zeros(1,3);
  
  // Prepare data ---
  
  L = TRobot;
  J = Joints;

  // Solve Direct Kinematics problem ---
  
  TCP0(1) = L(1)*cos(J(1)) + L(2)*cos(J(1)+J(2));
  TCP0(2) = L(1)*sin(J(1)) + L(2)*sin(J(1)+J(2));
  TCP0(3) = 0.0;
  
  return [Ret,TCP0];

endfunction

// =============================================================================

