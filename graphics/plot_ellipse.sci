// plot_ellipse.sci Draw an ellipsoid

function plot_ellipse(A, C)
    if size(A,1) ~= size(A,2)
        error('ellipse is defined by a square matrix');
    end

    if size(A,1) > 3
        error('can only plot ellipsoid for 2 or 3 dimenions');
    end
    nargin=argn(2);
    if nargin < 2
        C = zeros(1, size(A,1));
    end
    xc=C(1); yc=C(2); zc=C(3);
    
    eigA=spec(A);
    if min(real(eigA))<0 then
        error("A must be positive definite");
    end
    xr=eigA(1); yr=eigA(2); zr=eigA(3);
    [x,y,z]=ellipsoid(xc,yc,zc,xr,yr,zr,30);
    mesh(x,y,z);  
    h_axes = gca();
    h_axes.font_size = 2;
    h_axes.isoview = "on";
    h_axes.box = "off";

    // h_axes.rotation_angles = [63.5,-127];
    axscale = 1.05*max(real(eigA));
    xmin=-axscale;
    ymin=-axscale;
    zmin=-axscale;
    xmax=axscale;
    ymax=axscale;
    zmax=axscale;
    h_axes.data_bounds = [xmin,ymin,zmin; xmax,ymax,zmax];    
    
endfunction
