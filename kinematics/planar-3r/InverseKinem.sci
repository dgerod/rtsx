// =============================================================================
// p3r_InvKinem.sci
// =============================================================================

function [Ret,Joints] = p3rInvKinem (L, Pose)
//
// DESCRIPTION
//  Solves the Inverse Kinematics problem of a Planar-3R robot.
// PARAMETERS
//  L [IN] : Robot object [L1,L2,L3]
//  Pose [IN] : Pose [x,y,z,Rx,Ry,Rz]' in MCS.
//  Joints [OUT] : Position [j1,j2,j3]' in ACS.
// RETURN
//  Ret : Success (1) or error (<0)
// DEPENDS ON
//  None
// SEE ALSO
//  p3rDirKinem

    Ret = 1;
    Joints = zeros(2,3);
  
    // Prepare data ---

    l1 = L(1); 
    l2 = L(2);
    l3 = L(3);
    
    Rz = Pose(6);
    
    // x and y at the end of frame 2
    Wx = Pose(1) - l3*cos(Rz); 
    Wy = Pose(2) - l3*sin(Rz);
      
    // Solve Inverse Kinematics problem ---
      
    cosTheta2 = (Wx*Wx + Wy*Wy - l1*l1 - l2*l2) / (2*l1*l2);
      
    if abs(cosTheta2) > 1.0 then
        // Solution can not be reached	
        Ret = -1;
        return [Ret,Joints];
    else

        // Right Elbow Solution
        sinTheta2(1) = sqrt( 1-cosTheta2*cosTheta2 );
        // Left Elbow Solution
        sinTheta2(2) = -sinTheta2(1);

        J2(1) = atan( sinTheta2(1),cosTheta2 );
        J2(2) = atan( sinTheta2(2),cosTheta2 );

        k1 = l1 + l2*cosTheta2;
        k2(1) = l2*sinTheta2(1);
        k2(2) = l2*sinTheta2(2);
        
        aux1(1) = k1*Wy - k2(1)*Wx;
        aux1(2) = k1*Wy - k2(2)*Wx;
        aux2(1) = k1*Wx + k2(1)*Wy;
        aux2(2) = k1*Wx + k2(2)*Wy;
  
        J1(1) = atan(aux1(1),aux2(1));
        J1(2) = atan(aux1(2),aux2(2));
               
    end
  
    J3(1) = Rz - J1(1) - J2(1);
    J3(2) = Rz - J1(2) - J2(2);
    
    // J3 must be inside (-pi,pi]
    // (For J1 and J2 this is true because they are calculated with atan2)
    numTurns = int( ceil( (J3(1) + %pi)/(2*%pi) ) ) - 1;
    J3(1) = J3(1) - numTurns*(2*%pi);
    
    numTurns = int( ceil( (J3(2) + %pi)/(2*%pi) ) ) - 1;
    J3(2) = J3(2) - numTurns*(2*%pi);
    

    // Return the two solutions
    // Right solution
    Joints(1,:) = [J1(1),J2(1),J3(1)];
    // Left solution
    Joints(2,:) = [J1(2),J2(2),J3(2)];
    
    Ret = 1;
    return [Ret,Joints];

endfunction

// =============================================================================
