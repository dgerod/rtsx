//MSTRAJ  Multi-segment multi-axis trajectory
//
// TRAJ = MSTRAJ(P, QDMAX, Q0, DT, TACC, OPTIONS) is a multi-segment trajectory (KxN)
// based on via points P (MxN) and axis velocity limits QDMAX (1xN).  The 
// path comprises linear segments with polynomial blends.  The output 
// trajectory matrix has one row per time step, and one column per axis.
//
// - P (MxN) is a matrix of via points, 1 row per via point, one column 
//   per axis.  The last via point is the destination.
// - QDMAX (1xN) are axis velocity limits which cannot be exceeded, or
// - QDMAX (Mx1) are the durations for each of the M segments
// - Q0 (1xN) are the initial axis coordinates
// - DT is the time step
// - TACC (1x1) this acceleration time is applied to all segment transitions
// - TACC (1xM) acceleration time for each segment, TACC(i) is the acceleration 
//   time for the transition from segment i to segment i+1.  TACC(1) is also 
//   the acceleration time at the start of segment 1.
//
// TRAJ = MSTRAJ(SEGMENTS, QDMAX, Q0, DT, TACC, 'QD0',qd0, 'QDF', qdf, OPTIONS) as above but 
// additionally specifies the initial and final axis velocities (1xN).
//
// Options::
// 'verbose'    Show details.
// 'plot' Show trajectory plot
//
// Notes::
// - 
// - The path length K is a function of the number of via points, Q0, DT
//   and TACC.
// - The final via point P(M,:) is the destination.
// - The motion has M segments from Q0 to P(1,:) to P(2,:)  to P(M,:).
// - All axes reach their via points at the same time.
// - Can be used to create joint space trajectories where each axis is a joint
//   coordinate.
// - Can be used to create Cartesian trajectories with the "axes" assigned
//   to translation and orientation in RPY or Euler angle form.
//
// See also MTRAJ, LSPB, CTRAJ.

//
// This file is adapted from The Robotics Toolbox for Matlab (RTB).
// Copyright (C) 1993-2011, by Peter I. Corke
// 

function [TG, taxis]  = mstraj(segments, qdmax, tsegment, q, dt, Tacc, varargin)

    varnum=length(varargin);  // number of arguments

   opt.plot = 0;
   opt.verbose = 0;
    ns = numrows(segments);
    nj = numcols(segments);

    if ~isempty(qdmax) & ~isempty(tsegment)
        error('Can only specify one of qdmax or tsegment');
    end
    if isempty(qdmax) & isempty(tsegment)
        error('Must specify one of qdmax or tsegment');
    end

   qd0 = zeros(1,nj);
   qdf = zeros(1,nj);
   
    for iv =1:varnum
        if type(varargin(iv)==10) then
            varargin(iv)=convstr(varargin(iv),'l');    // to lower case 
        end
       if varargin(iv)=='verbose' then
           opt.verbose = 1;
       elseif varargin(iv)=='plot' then // show graphics
           opt.plot = 1;     
       elseif varargin(iv)=='qd0'& (iv<varnum) then
           qd0 = varargin(iv+1);
       elseif varargin(iv)=='qdf'& (iv<varnum) then
           qdf = varargin(iv+1);           
       end      
    end    // for iv=1:varnum

    //[opt,args] = tb_optparse([], varargin);

    //if length(args) > 0
   //        qd0 = args{1};
   //    else
   //        qd0 = zeros(1, nj);
   //    end
   //    if length(args) > 1
   //        qdf = args{2};
   //    else
   //        qdf = zeros(1, nj);
   //    end
   //
    // set the initial conditions
    q_prev = q;
    qd_prev = qd0;

    clock = 0;      // keep track of time
    arrive = [];    // record planned time of arrival at via points

    tg = [];
    taxis = [];
    //pause;
    for seg=1:ns
        if opt.verbose
            printf('------------------- segment %d\n', seg);
        end

        // set the blend time, just half an interval for the first segment

        if length(Tacc) > 1
            tacc = Tacc(seg);
        else
            tacc = Tacc;
        end

        tacc = ceil(tacc/dt)*dt;
        tacc2 = ceil(tacc/2/dt) * dt;
        if seg == 1
            taccx = tacc2;
        else
            taccx = tacc;
        end

        // estimate travel time
        //    could better estimate distance travelled during the blend
        q_next = segments(seg,:);    // current target
        dq = q_next - q_prev;    // total distance to move this segment

        //// probably should iterate over the next section to get qb right...
        // while 1
        //   qd_next = (qnextnext - qnext)
        //   tb = abs(qd_next - qd) ./ qddmax;
        //   qb = f(tb, max acceleration)
        //   dq = q_next - q_prev - qb
        //   tl = abs(dq) ./ qdmax;

        if ~isempty(qdmax)
            // qdmax is specified, compute slowest axis

            qb = taccx * qdmax / 2;        // distance moved during blend
            tb = taccx;

            // convert to time
            tl = abs(dq) ./ qdmax;
            //tl = abs(dq - qb) ./ qdmax;
            tl = ceil(tl/dt) * dt;

            // find the total time and slowest axis
            tt = tb + tl;
            [tseg,slowest] = max(tt);
            taxis(seg,:) = tt;

            // best if there is some linear motion component
            if tseg <= 2*tacc
                tseg = 2 * tacc;
            end
        elseif ~isempty(tsegment)
            // segment time specified, use that
            tseg = tsegment(seg);
            slowest = %nan;
        end

        // log the planned arrival time
        arrive(seg) = clock + tseg;
        if seg > 1
            arrive(seg) = arrive(seg) + tacc2;
        end

        if opt.verbose
            printf('seg %d, slowest axis %d, time required %.4g\n', ...
                seg, slowest, tseg);
        end

        //// create the trajectories for this segment

        // linear velocity from qprev to qnext
        if tseg==0 then
            tseg = %eps;  // avoid divide by zero error
        end
        qd = dq / tseg;

        // add the blend polynomial
        qb = jtraj(q, q_prev+tacc2*qd, 0:dt:taccx, qd_prev, qd);
        tg = [tg; qb(2:size(qb,1),:)];

        clock = clock + taccx;     // update the clock

        // add the linear part, from tacc/2+dt to tseg-tacc/2
        for t=tacc2+dt:dt:tseg-tacc2
            s = t/tseg;
            q = (1-s) * q_prev + s * q_next;       // linear step
            tg = [tg; q];
            clock = clock + dt;
        end

        q_prev = q_next;    // next target becomes previous target
        qd_prev = qd;
    end
    // add the final blend
    qb = jtraj(q, q_next, 0:dt:tacc2, qd_prev, qdf);
   tg = [tg; qb(2:size(qb,1),:)];
   TG = tg;

    // plot a graph if 'plot' option is specified
    if opt.plot
        t = (0:numrows(tg)-1)'*dt;
        figure;
        plot(t, tg, '-o');
        
        plot(arrive, segments, 'bo', 'MarkerFaceColor', 'k');
        
        xgrid;
        xlabel('time');
        title('Multi-segment multi-axis trajectory');
        //xaxis(t(1), t(end))

    end
endfunction





