// GeoMakeJointAxis.sci  make data to plot joint axis
// www.controlsystemslab.com   July 2012

function JAxisData=GeoMakeJointAxis(Location,Orientation,Length)
    r = Location;
    R = Orientation;
    JAxisData(1,:)= r + [0 0 -Length/2]*R';
    JAxisData(2,:)= r + [0 0 Length/2]*R';
endfunction