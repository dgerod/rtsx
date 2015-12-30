// =============================================================================
// scdh_DirKinem.sci
// =============================================================================

function [Ret,TCP0] = schDirKinem (MatrixDH)
//
// DESCRIPTION
//  Solves the Direct Kinematics problem of a Scara robot with DH parameters. 
// PARAMETERS
//  MatrixDH [IN] : array of DH parameters, structure ('alfa','a','theta','d');
//  TCP0  [OUT] : Pose [x,y,z]' in MCS.
// RETURN
//  Ret : Success (1) or error (<0)
// DEPENDS ON
//  None
// SEE ALSO
//

    Ret = 1;
    TCP0 = zeros(1,3);
    
    if 1==0 then
        matrixA_1to0=zeros(4,4);
        matrixA_2to1=zeros(4,4);
        matrixA_3to2=zeros(4,4);
        matrixA=zeros(4,4);
        
        for i=1:3
            CosI=cos(MatrixDH(i).theta);
            SinI=sin(MatrixDH(i).theta);
        
            matrixA(1,1)=CosI;
            matrixA(1,2)=-SinI;
            matrixA(2,1)=SinI;
            matrixA(2,2)=CosI;
            matrixA(3,3)=1;
            matrixA(1,4)=MatrixDH(i).a*CosI;
            matrixA(2,4)=MatrixDH(i).a*SinI;
            matrixA(3,4)=MatrixDH(i).d;
            matrixA(4,4)=1;
            
            select i
            case 1
                matrixA_1to0=matrixA;
            case 2
                matrixA_2to1=matrixA;
            case 3
                matrixA_3to2=matrixA;
            end
            
        end
        
        matrixT_3to0=matrixA_1to0*matrixA_2to1*matrixA_3to2;
        TCP0=matrixT_3to0(1:3,4)
   
    else
        TCP0(1) = MatrixDH(1).a*cos(MatrixDH(1).theta) + MatrixDH(2).a*cos(MatrixDH(1).theta + MatrixDH(2).theta) + MatrixDH(3).a*cos(MatrixDH(1).theta + MatrixDH(2).theta + MatrixDH(3).theta);
        TCP0(2) = MatrixDH(1).a*sin(MatrixDH(1).theta) + MatrixDH(2).a*sin(MatrixDH(1).theta + MatrixDH(2).theta) + MatrixDH(3).a*sin(MatrixDH(1).theta + MatrixDH(2).theta + MatrixDH(3).theta);
        TCP0(3) = MatrixDH(1).d + MatrixDH(2).d + MatrixDH(3).d;
    end
    
    Ret = 1;
    return [Ret,TCP0];

endfunction

// =============================================================================
