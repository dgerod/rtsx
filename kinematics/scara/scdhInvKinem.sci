// =============================================================================
// scdh_InvKinem.sci
// =============================================================================

function [Ret,Joints] = schInvKinem (MatrixDH, PosEndEffector, IsLeftSolution, IsRRP)
//
// DESCRIPTION
//  Solves the Inverse Kinematics problem of a Scara robot with DH parameters. 
// PARAMETERS
//  MatrixDH [IN] : array of DH parameters, structure ('alfa','a','theta','d');
//  PosEndEffector  [IN] : Pose [x,y,z]' in MCS.
//  IsLeftSolution [IN] : if true output is left solution
//  Joints [OUT] : Joints
// RETURN
//  Ret : Success (1) or error (<0)
// DEPENDS ON
//  None
// SEE ALSO
//

    Ret = 1;
    Joints = zeros(2,3);
    // index of joints
    if IsRRP==1 then
        indexTheta1=1;
        indexTheta2=2; 
        indexTheta3=3; 
        indexD1=1;
        indexD2=2;
        indexD3=3;
        
        Px=PosEndEffector(1);
        Py=PosEndEffector(2);
        Pz=PosEndEffector(3);
    else
        indexTheta1=2;
        indexTheta2=3;        
        indexTheta3=1;
        indexD1=2;
        indexD2=3;
        indexD3=1;

        Px=PosEndEffector(1)-MatrixDH(indexD3).a;
        Py=PosEndEffector(2);
        Pz=PosEndEffector(3);
    end
  
  
    cosTheta2 = (Px*Px + Py*Py - MatrixDH(indexTheta1).a*MatrixDH(indexTheta1).a - MatrixDH(indexTheta2).a*MatrixDH(indexTheta2).a) / (2*MatrixDH(indexTheta1).a*MatrixDH(indexTheta2).a);
    if abs(cosTheta2) > 1.0 then
        // Solution can not be reached
        Ret = -1;
        return [Ret,Joints];
    else
        sinTheta2 = sqrt( 1-cosTheta2*cosTheta2 );
        if IsLeftSolution==1 then
            // Left Elbow Solution
            sinTheta2 = -sinTheta2;
        else
            // Right Elbow Solution
            //sinTheta2 = sinTheta2;
        end
        J2 = atan( sinTheta2,cosTheta2 );

        k1 = MatrixDH(indexTheta1).a + MatrixDH(indexTheta2).a*cosTheta2;
        k2 = MatrixDH(indexTheta2).a*sinTheta2;
        
        aux1=k1*Py-k2*Px;
        aux2=k1*Px+k2*Py;
  
        J1=atan(aux1,aux2);
  
    end
    J3=Pz - MatrixDH(indexD1).d - MatrixDH(indexD2).d;
    
    if IsRRP==1 then
        Joints=[J1, J2, J3];        
    else
        Joints=[J3, J1, J2];    
    end

    Ret = 1;
    return [Ret,Joints];

endfunction
