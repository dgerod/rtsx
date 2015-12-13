// =============================================================================
// p2r_DirKinem.sci
// =============================================================================

function [Ret,TCP0] = p2rDirKinem (TRobot,Joints)
//
// DESCRIPTION
//  Solves the Direct Kinematics problem of a Planar-2R robot.
// PARAMETERS
//  TRobot [IN] : Robot object [L1,L2]
//  Joints [IN] : Position [j1,j2]' in JCS.
//  TCP0  [OUT] : Pose [x,y,z]' in MCS.
// RETURN
//  Ret : Success (1) or error (<0)
// DEPENDS ON
//  None
// SEE ALSO
//  p2rInvKinem
//

  Ret = 1;
  TCP0 = zeros(1,3);
  
  // Prepare data ---
  
  L = TRobot;
  J = Joints;

  // Solve Direct Kinematics problem ---
  
  TCP0(1) = L(1)*cos(J(1)) + L(2)*cos(J(1)+J(2));
  TCP0(2) = L(1)*sin(J(1)) + L(2)*sin(J(1)+J(2));
  TCP0(3) = 0.0;
  
  // ---

  return [Ret,TCP0];

endfunction

// =============================================================================

