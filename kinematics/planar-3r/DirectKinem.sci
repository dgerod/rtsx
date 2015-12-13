// =============================================================================
// p3r_DirKinem.sci
// =============================================================================

function [Ret,Pose] = p3rDirKinem (L, Joints)
//
// DESCRIPTION
//  Solves the Direct Kinematics problem of a Planar-3R robot. 
// PARAMETERS
//  L [IN] : Robot object [L1,L2,L3]
//  Joints [IN] : Position [j1,j2,j3]' in ACS.
//  Pose [OUT] : Pose [x,y,z,Rx,Ry,Rz]' in MCS.
// RETURN
//  Ret : Success (1) or error (<0)
// DEPENDS ON
//  None
// SEE ALSO
//

    Pose = zeros(1,6);
   
    Pose(1) = L(1)*cos(Joints(1)) + L(2)*cos(Joints(1) + Joints(2)) + ...
              L(3)*cos(Joints(1) + Joints(2) + Joints(3));
    Pose(2) = L(1)*sin(Joints(1)) + L(2)*sin(Joints(1) + Joints(2)) + ...
              L(3)*sin(Joints(1) + Joints(2) + Joints(3));
    Pose(3) = 0.0;
    Pose(4) = 0.0;
    Pose(5) = 0.0;
    Pose(6) = Joints(1) + Joints(2) + Joints(3);
    
    // Rz must be inside (-pi,pi]
    numTurns = int( ceil( (Pose(6) + %pi)/(2*%pi) ) ) - 1;
    Pose(6) = Pose(6) - numTurns*(2*%pi);

    Ret = 1;
    return [Ret,Pose];

endfunction

// =============================================================================
