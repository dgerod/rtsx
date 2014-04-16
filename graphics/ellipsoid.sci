// ellipsoid.sci 
//ELLIPSOID Generate ellipsoid.
//
// [X,Y,Z]=ELLIPSOID(XC,YC,ZC,XR,YR,ZR,N) generates three
// (N+1)-by-(N+1) matrices so that SURF(X,Y,Z) produces an
// ellipsoid with center (XC,YC,ZC) and radii XR, YR, ZR.
// 
// [X,Y,Z]=ELLIPSOID(XC,YC,ZC,XR,YR,ZR) uses N = 20.
//
// ELLIPSOID(...) and ELLIPSOID(...,N) with no output arguments
// graph the ellipsoid as a SURFACE and do not return anything.
//
// The ellipsoidal data is generated using the equation:
//
//  (X-XC)^2     (Y-YC)^2     (Z-ZC)^2
//  --------  +  --------  +  --------  =  1
//    XR^2         YR^2         ZR^2

function [x, y, z] = ellipsoid(xc, yc, zc, xr, yr, zr, n)


    theta = (-n:2:n)/n*%pi;
    phi = (-n:2:n)'/n*%pi/2;
    cosphi = cos(phi); 
    cosphi(1) = 0; 
    cosphi(n+1) = 0;
    sintheta = sin(theta); 
    sintheta(1) = 0; sintheta(n+1) = 0;
    
    x = cosphi*cos(theta);
    y = cosphi*sintheta;
    z = sin(phi)*ones(1,n+1);
    
    x = xr*x+xc;
    y = yr*y+yc;
    z = zr*z+zc;


endfunction

