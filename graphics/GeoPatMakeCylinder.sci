// GeoPatMakeCylinder.sci

function [PatchData1_X,PatchData1_Y,PatchData1_Z,PatchData2_X,PatchData2_Y,PatchData2_Z] = GeoPatMakeCylinder(VertexData)
    n_side = size(VertexData,1)/2;
    // Side Patches
    for i_pat=1:n_side-1
        Index_Patch1(i_pat,:) = [i_pat,i_pat+1,i_pat+1+n_side,i_pat+n_side];
    end
    Index_Patch1(n_side,:) = [n_side,1,1+n_side,2*n_side];

    for i_pat=1:n_side

        // Side patches data
        PatchData1_X(:,i_pat) = VertexData(Index_Patch1(i_pat,:),1);
        PatchData1_Y(:,i_pat) = VertexData(Index_Patch1(i_pat,:),2);
        PatchData1_Z(:,i_pat) = VertexData(Index_Patch1(i_pat,:),3);
    end
    // Bottom Patches
    Index_Patch2(1,:) = [1:n_side];
    Index_Patch2(2,:) = [n_side+1:2*n_side];

    for i_pat=1:2

        // Bottom patches data
        PatchData2_X(:,i_pat) = VertexData(Index_Patch2(i_pat,:),1);
        PatchData2_Y(:,i_pat) = VertexData(Index_Patch2(i_pat,:),2);
        PatchData2_Z(:,i_pat) = VertexData(Index_Patch2(i_pat,:),3);
    end


endfunction