function VertexData = GeoVerMakeCylinder(Location,Orientation,Radius,Height,SideCount)
    r = Location;
    R = Orientation;
    // Vertices
    n_side = SideCount;


    for i_ver=1:n_side
        VertexData_0(i_ver,:) = [Radius*cos(2*%pi/n_side*i_ver),Radius*sin(2*%pi/n_side*i_ver),-Height/2];
        VertexData_0(n_side+i_ver,:) = [Radius*cos(2*%pi/n_side*i_ver),Radius*sin(2*%pi/n_side*i_ver),Height/2];
    end

    n_ver = 2*n_side;

    for i_ver=1:n_ver
        VertexData(i_ver,:) = r + VertexData_0(i_ver,:)*R';
    end
    
endfunction 