//LSPB  Linear segment with parabolic blend
// www.controlsystemslab.com  August 2012
//
// [S,SD,SDD] = LSPB(S0, SF, M) is a scalar trajectory (Mx1) that varies 
// smoothly from S0 to SF in M steps using a constant velocity segment and 
// parabolic blends (a trapezoidal path).  Velocity and acceleration can be
// optionally returned as SD (Mx1) and SDD (Mx1).
//
// [S,SD,SDD] = LSPB(S0, SF, M, V) as above but specifies the velocity of 
// the linear segment which is normally computed automatically.
//
// [S,SD,SDD] = LSPB(S0, SF, T) as above but specifies the trajectory in 
// terms of the length of the time vector T (Mx1).
//
// [S,SD,SDD] = LSPB(S0, SF, T, V) as above but specifies the velocity of 
// the linear segment which is normally computed automatically and a time
// vector.
//
// Notes::
// - lspbplot( ) behaves like lspb( ) but also plot the trajectory
// - For some values of V no solution is possible and an error is flagged.
//
// See also QPOLY, JTRAJ.
//
// This file is adapted from The Robotics Toolbox for Matlab (RTB).
// Copyright (C) 1993-2011, by Peter I. Corke
// 

function [s,sd,sdd] = lspb(q0, q1, t, V)

    nargin=argn(2);
    nargout = argn(1);
       
    t0 = t;
    if isscalar(t)
        t = (0:t-1)';
    else
        t = t(:);
    end

    tf = max(t(:));

    if nargin < 4
        // if velocity not specified, compute it
        V = (q1-q0)/tf * 1.5;
    else
        if V <= (q1-q0)/tf
            error('V too small');
        elseif V > 2*(q1-q0)/tf
            error('V too big');
        end
    end

    if q0 == q1
        s = ones(size(t)) * q0;
        sd = zeros(size(t));
        sdd = zeros(size(t));
        return
    end

    tb = (q0 - q1 + V*tf)/V;
    a = V/tb;
    
    p = zeros(length(t), 1);
    pd = p;
    pdd = p;
    
    for i = 1:length(t)
        tt = t(i);

        if tt <= tb
            // initial blend
            p(i) = q0 + a/2*tt^2;
            pd(i) = a*tt;
            pdd(i) = a;
        elseif tt <= (tf-tb)
            // linear motion
            p(i) = (q1+q0-V*tf)/2 + V*tt;
            pd(i) = V;
            pdd(i) = 0;
        else
            // final blend
            p(i) = q1 - a/2*tf^2 + a*tf*tt - a/2*tt^2;
            pd(i) = a*tf - a*tt;
            pdd(i) = -a;
        end
    end

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


function [s,sd,sdd] = lspbplot(q0, q1, t, V)

    nargin=argn(2);
    nargout = argn(1);
       
    t0 = t;
    if isscalar(t)
        t = (0:t-1)';
    else
        t = t(:);
    end

    tf = max(t(:));

    if nargin < 4
        // if velocity not specified, compute it
        V = (q1-q0)/tf * 1.5;
    else
        if V < (q1-q0)/tf
            error('V too small');
        elseif V > 2*(q1-q0)/tf
            error('V too big');
        end
    end

    if q0 == q1
        s = ones(size(t)) * q0;
        sd = zeros(size(t));
        sdd = zeros(size(t));
        return
    end

    tb = (q0 - q1 + V*tf)/V;
    a = V/tb;

    p = zeros(length(t), 1);
    pd = p;
    pdd = p;
    
    for i = 1:length(t)
        tt = t(i);

        if tt <= tb
            // initial blend
            p(i) = q0 + a/2*tt^2;
            pd(i) = a*tt;
            pdd(i) = a;
        elseif tt <= (tf-tb)
            // linear motion
            p(i) = (q1+q0-V*tf)/2 + V*tt;
            pd(i) = V;
            pdd(i) = 0;
        else
            // final blend
            p(i) = q1 - a/2*tf^2 + a*tf*tt - a/2*tt^2;
            pd(i) = a*tf - a*tt;
            pdd(i) = -a;
        end
    end
    
    if isscalar(t0)
    // for scalar time steps, axis is labeled 1 .. M
          xt = t+1;
      else
          // for vector time steps, axis is labeled by vector M
          xt = t;
      end

      //clf
      figure;
      subplot(311)
      // highlight the accel, coast, decel phases with different
      // colored markers
      //hold on
      k = t<= tb;
      plot(xt(k), p(k), 'r-o');
      k = (t>=tb) & (t<= (tf-tb));
      plot(xt(k), p(k), 'b-o');
      k = t>= (tf-tb);
      plot(xt(k), p(k), 'g-o');
      xgrid; ylabel('s');
      //hold off
       xlabel('time');
        title('Linear segment with parabolic blend (LSPB) trajectory');

      subplot(312)
      plot(xt, pd); xgrid; ylabel('sd');
      h2=gca();
      h2.background = 33;
      vmax = h2.data_bounds(2,2);
      h2.data_bounds(2,2) = 1.05*vmax;
      xlabel('time');      

      subplot(313)
      plot(xt, pdd); xgrid; ylabel('sdd');
      h3=gca();

      h3.background = 33;
      xlabel('time');


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



