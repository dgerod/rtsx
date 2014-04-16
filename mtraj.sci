//MTRAJ Multi-axis trajectory between two points
// www.controlsystemslab.com  August 2012
//
// [Q,QD,QDD] = MTRAJ(Q0, QF, M, options) is a multi-axis trajectory (MxN) varying
// from state Q0 (1xN) to QF (1xN) according to the scalar trajectory function 
// (qpoly or lspb) in M steps. Joint velocity and acceleration can be optionally returned as 
// QD (MxN) and QDD (MxN) respectively.  The trajectory outputs have one row per 
// time step, and one column per axis.
//
// The shape of the trajectory is given by the scalar trajectory function TFUNC
//      [S,SD,SDD] = TFUNC(S0, SF, M);
// and possible values of TFUNC include lspb for a trapezoidal trajectory, or
//     qpoly for a polynomial trajectory.
//
// [Q,QD,QDD] = MTRAJ(Q0, QF, T, options) as above but specifies the trajectory 
// length in terms of the length of the time vector T (Mx1).
//
// Notes::
// - Q, QD, and QDD are plotted when 'plot' is specified.
// - When TFUNC is tpoly the result is functionally equivalent to JTRAJ except 
//   that no initial velocities can be specified. JTRAJ is computationally a little
//   more efficient.
//
// See also JTRAJ, MSTRAJ, LSPB, QPOLY.


// This file is adapted from The Robotics Toolbox for Matlab (RTB).
// Copyright (C) 1993-2011, by Peter I. Corke
//

function [S,Sd,Sdd] = mtraj(q0, qf, M, varargin)

    nargin=argn(2);
    nargout = argn(1);
    varnum=length(varargin);  // number of arguments
    uselspb = 1;      // default to lspb
    useqpoly = 0;
    usecpoly = 0;
    showplot = 0;
    for iv =1:varnum
        if varargin(iv)=='qpoly' then
            useqpoly = 1;
            uselspb = 0;
            usecpoly = 0;

        elseif varargin(iv)=='lspb' then
            uselspb = 1;
            useqpoly = 0;
            usecpoly = 0;
        elseif varargin(iv) == 'cpoly' then
            usecpoly = 1;
            useqpoly = 0;
            uselspb = 0;

       elseif varargin(iv)=='plot' then // show graphics
           showplot = 1;     
       end      
    end    // for iv=1:varnum


    M0 = M;
    if ~isscalar(M)
        M = length(M);
    end
    if numcols(q0) ~= numcols(qf)
        error('must be same number of columns in q0 and qf')
    end

    s = zeros(M, numcols(q0));
    sd = zeros(M, numcols(q0));
    sdd = zeros(M, numcols(q0));
   //pause;
    for i=1:numcols(q0)
        // for each axis
        if  useqpoly
           [stmp, sdtmp, sddtmp] = qpoly(q0(i), qf(i), M);
        elseif usecpoly
           [stmp, sdtmp, sddtmp] = cpoly(q0(i), qf(i), M);
        elseif uselspb
           [stmp, sdtmp, sddtmp] = lspb(q0(i), qf(i), M);            
        end
        s(:,i) = stmp;
        sd(:,i) = sdtmp;
        sdd(:,i) = sddtmp;
    end

// - If no output arguments are specified S, SD, and SDD are plotted 
//   against time.

    select nargout

        case 1
            S = s;
        case 2
            S = s;
            Sd = sd;
        case 3
            S = s;
            Sd = sd;
            Sdd = sdd;
    end
   if showplot
       if isscalar(M0)
          t = [1:M0]';
       else
          t = M0;
       end
       figure;
       subplot(311)
       plot(t, s); xgrid; ylabel('s');
       xlabel('time');
       if uselspb 
           title('Linear segment parabolic blend trajectory');
       elseif useqpoly
           title('Quintic polynomial trajectory');
       elseif usecpoly
           title('Cubic polynomial trajectory');
       end
       subplot(312)
       plot(t, sd); xgrid; ylabel('sd');
       h2=gca();
       h2.background = 33;
       xlabel('time');
       //title('velocity');
       subplot(313)
       plot(t, sdd); xgrid; ylabel('sdd');
       h3=gca();
       h3.background = 33;
       xlabel('time');
   end

   //pause;
endfunction







