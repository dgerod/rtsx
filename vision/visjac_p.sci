// visjac_p.sci
//Visual motion Jacobian for point feature
//
// J = visjac_p(cam, UV, Z) is the image Jacobian (2Nx6) for the image plane 
// points UV (2xN).  The depth of the points from the camera is given by Z
// which is a scalar for all points, or a vector (Nx1) of depth for each point.
//
// The Jacobian gives the image-plane point velocity in terms of camera spatial
// velocity. 
//
// This file is adapted from The Robotics Toolbox for Matlab (RTB).
// Copyright (C) 1993-2011, by Peter I. Corke
//


function L = visjac_p(cam, uv, Z)
    
    if numcols(uv) > 1
        L = [];
        if length(Z) == 1
            // if depth is a scalar, assume same for all points
            Z = repmat(Z, 1, numcols(uv));
        end
        // recurse for each point
        for i=1:numcols(uv)
            L = [L; visjac_p(cam, uv(:,i), Z(i))];
        end
        return;
    end
    
    // convert to normalized image-plane coordinates
    x = (uv(1) - cam.u0) * cam.rho(1) / cam.f;
    y = (uv(2) - cam.v0) * cam.rho(2) / cam.f;

    L = [
        1/Z, 0, -x/Z, -x*y, (1+x^2), -y
        0, 1/Z, -y/Z, -(1+y^2), x*y, x
        ];

    L = -cam.f * diag(1 ./cam.rho) * L;
endfunction
