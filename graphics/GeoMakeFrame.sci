// GeoMakeFrame.sci returns coordiante frame data
// www.controlsystemslab.com  July 2012

function [FrameDataX,FrameDataY,FrameDataZ]=GeoMakeFrame(Location,Orientation,AxLength)
    r = Location;
    R = Orientation;
    
    AxisData0 = AxLength*[1 0 0;
                0 1 0;
                0 0 1];
    for i=1:3
        AxisData(i,:)=r + AxisData0(i,:)*R';
    end
    FrameDataX = [r(1)             r(1)         r(1);
                  AxisData(1,1)   AxisData(2,1) AxisData(3,1)];
    FrameDataY = [r(2)             r(2)         r(2);
                  AxisData(1,2)   AxisData(2,2) AxisData(3,2)];
    FrameDataZ = [r(3)             r(3)         r(3);
                  AxisData(1,3)   AxisData(2,3) AxisData(3,3)];

endfunction
