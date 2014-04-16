function VertexData = GeoVerMakeBlock(Location,Orientation,SideLength)

r = Location;
R = Orientation;

Lx = SideLength(1);
Ly = SideLength(2);
Lz = SideLength(3);


// Vertices
//VertexData_0 = [Lx*ones(8,1), Ly*ones(8,1), Lz*ones(8,1)]...
//    .*[0,0,0;
//    1,0,0;
//    0,1,0;
//    0,0,1;
//    1,1,0;
//    0,1,1;
//    1,0,1;
//    1,1,1];

// Vertices
VertexData_0 =     [0,0,0;
    1,0,0;
    0,1,0;
    0,0,1;
    1,1,0;
    0,1,1;
    1,0,1;
    1,1,1]-0.5*ones(8,3);

VertexData_0 = [Lx*ones(8,1), Ly*ones(8,1), Lz*ones(8,1)].*VertexData_0;
n_ver = 8;

for i_ver=1:n_ver
    
    VertexData(i_ver,:) = r + VertexData_0(i_ver,:)*R';
end

endfunction