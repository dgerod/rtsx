//QPOLY Generate scalar polynomial trajectory using quintic polynomial
//
// [S,SD,SDD] = TPOLY(S0, SF, M) is a scalar trajectory (Mx1) that varies 
// smoothly from S0 to SF in M steps using a quintic (5th order) polynomial.
// Velocity and acceleration can be optionally returned as SD (Mx1) and SDD (Mx1).
//
// [S,SD,SDD] = TPOLY(S0, SF, T) as above but specifies the trajectory in 
// terms of the length of the time vector T (Mx1).
//
// Notes::
// - If no output arguments are specified S, SD, and SDD are plotted.
//

// [S,SD,SDD] = TPOLY(S0, SF, N, SD0, SDF) as above but specifies initial 
// and final joint velocity for the trajectory.
//
// [S,SD,SDD] = TPOLY(S0, SF, T, SD0, SDF) as above but specifies initial 
// and final joint velocity for the trajectory and time vector T.
//
// Notes::
// - In all cases if no output arguments are specified S, SD, and SDD are plotted 
//   against time.
//
// See also LSPB, JTRAJ.

// This file is adapted The Robotics Toolbox for Matlab (RTB).
// Copyright (C) 1993-2011, by Peter I. Corke 


function [s,sd,sdd] = qpoly(q0, qf, t, qd0, qdf)

    t0 = t;
    nargin=argn(2);
    nargout = argn(1);
    if isscalar(t)
		t = (0:t-1)';
    else
        t = t(:);
    end
    if nargin < 4
        qd0 = 0;
    end
    if nargin < 5
        qdf = 0;
    end
    
    tf = max(t);
    
    // solve for the polynomial coefficients using least squares
    X = [
        0           0           0         0       0   1
        tf^5        tf^4        tf^3      tf^2    tf  1
        0           0           0         0       1   0
        5*tf^4      4*tf^3      3*tf^2    2*tf    1   0
        0           0           0         2       0   0
        20*tf^3     12*tf^2     6*tf      2       0   0
    ];
    b = [q0 qf qd0 qdf 0 0]';
    //coeffs = (X \ b)';
    //coeffs = lsq(X,b)';
    coeffs = (inv(X)*b)';    
    // coefficients of derivatives 
    coeffs_d = coeffs(1:5) .* (5:-1:1);
    coeffs_dd = coeffs_d(1:4) .* (4:-1:1);

    // evaluate the polynomials
    p = polyval(coeffs, t);
    pd = polyval(coeffs_d, t);
    pdd = polyval(coeffs_dd, t);

    select nargout

        case 1
            s = p;
        case 2
            s = p;
            sd = pd;
        case 3
            s = p;
            sd = pd;
            sdd = pdd;
    end
    
endfunction

function [s,sd,sdd] = qpolyplot(q0, qf, t, qd0, qdf)
    t0 = t;
    nargin=argn(2);
    nargout = argn(1);
    if isscalar(t)
		t = (0:t-1)';
    else
        t = t(:);
    end
    if nargin < 4
        qd0 = 0;
    end
    if nargin < 5
        qdf = 0;
    end
    
    tf = max(t);
    
    // solve for the polynomial coefficients using least squares
    X = [
        0           0           0         0       0   1
        tf^5        tf^4        tf^3      tf^2    tf  1
        0           0           0         0       1   0
        5*tf^4      4*tf^3      3*tf^2    2*tf    1   0
        0           0           0         2       0   0
        20*tf^3     12*tf^2     6*tf      2       0   0
    ];
    b = [q0 qf qd0 qdf 0 0]';

   // disp(b);
    //coeffs = (X \ b)';
    //coeffs = lsq(X,b)';
    coeffs = (inv(X)*b)';
    // coefficients of derivatives 
    coeffs_d = coeffs(1:5) .* (5:-1:1);
    coeffs_dd = coeffs_d(1:4) .* (4:-1:1);

    // evaluate the polynomials
    p = polyval(coeffs, t);
    pd = polyval(coeffs_d, t);
    pdd = polyval(coeffs_dd, t);
   

    if isscalar(t0)
        // for scalar time steps, axis is labeled 1 .. M
        xt = t+1;
    else
        // for vector time steps, axis is labeled by vector M
        xt = t;
    end
   //pause;
    figure;
    subplot(311)
    plot(xt, p); xgrid; ylabel('s');
    xlabel('time');
    title('Quintic polynomial trajectory');
    subplot(312)
    plot(xt, pd); xgrid; ylabel('sd');
    h2=gca();
    h2.background = 33;
    xlabel('time');
    //title('velocity');
    subplot(313)
    plot(xt, pdd); xgrid; ylabel('sdd');
    h3=gca();
    h3.background = 33;
    xlabel('time');
    //title('acceleration');
    select nargout
        case 1
            s = p;
        case 2
            s = p;
            sd = pd;
        case 3
            s = p;
            sd = pd;
            sdd = pdd;
    end
endfunction


function [s,sd,sdd] = tpoly(q0, qf, t, qd0, qdf) // for rvc compatibility

    t0 = t;
    nargin=argn(2);
    nargout = argn(1);
    if isscalar(t)
		t = (0:t-1)';
    else
        t = t(:); 
    end
    if nargin < 4
        qd0 = 0;
    end
    if nargin < 5
        qdf = 0;
    end
   [s,sd,sdd]=qpoly(q0,qf,t,qd0,qdf); 

endfunction






