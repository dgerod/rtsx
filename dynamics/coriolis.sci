//Coriolis.sci Coriolis matrix
//
// C = coriolis(robot, Q, QD) is the Coriolis/centripetal matrix (NxN) for
// the robot in configuration Q and velocity QD, where N is the number of
// joints.  The product C*QD is the vector of joint force/torque due to velocity
// coupling.  The diagonal elements are due to centripetal effects and the 
// off-diagonal elements are due to Coriolis effects.  This matrix is also 
// known as the velocity coupling matrix, since gives the disturbance forces
// on all joints due to velocity of any joint.
//
// If Q and QD are matrices (KxN), each row is interpretted as a joint state 
// vector, and the result (NxNxK) is a 3d-matrix where each plane corresponds
// to a row of Q and QD.
//
// C = coriolis(robot, QQD) as above but the matrix QQD (1x2N) is [Q QD].
//
// Notes::
// - Joint friction is also a joint force proportional to velocity but it is
//   eliminated in the computation of this value.
// - Computationally slow, involves N^2/2 invocations of RNE.
//
// See also rne()
//
// This file is adapted from The Robotics Toolbox for Matlab (RTB).
// Copyright (C) 1993-2011, by Peter I. Corke
// 


function C = coriolis(robot, q, qd)
    nargin = argn(2);
    n = robot.nj;

    if nargin == 2
        // coriolis( [q qd] )
        if numcols(q) ~= 2*n
            msg = sprintf('argument [q qd] must have %d columns', 2*n);
            error(msg);
        end
        qd = q(:,n+1:end);
        q = q(:,1:n);
    else
        if numcols(q) ~= n
            msg = sprintf('argument q must have %d columns', n);
            error(msg);
        end
        if numcols(qd) ~= n
            msg = sprintf('argument qd must have %d columns', n);
        end
    end

    // we need to create a clone robot with no friciton, since friction
    // is also proportional to joint velocity
    robot2 = nofriction(robot,'all');

    if numrows(q) > 1
        if numrows(q) ~= numrows(qd)
            error('for trajectory, q and qd must have same number of rows');
        end
        C = [];
        for i=1:numrows(q)
            C = cat(3, C, coriolis(robot2, q(i,:), qd(i,:)));
        end
        return
    end

    N = robot2.nj;
    C = zeros(N,N);
    Csq = zeros(N,N);

    // find the torques that depend on a single finite joint speed,
    // these are due to the squared (centripetal) terms
    //
    //  set QD = [1 0 0 ...] then resulting torque is due to qd_1^2
    for j=1:N
        QD = zeros(1,N);
        QD(j) = 1;
        tau = rne(robot2, q, QD, _zeros(size(q)), [0 0 0]');
        Csq(:,j) = Csq(:,j) + tau';
    end

    // find the torques that depend on a pair of finite joint speeds,
    // these are due to the product (Coridolis) terms
    //  set QD = [1 1 0 ...] then resulting torque is due to 
    //    qd_1 qd_2 + qd_1^2 + qd_2^2
    for j=1:N
        for k=j+1:N
            // find a product term  qd_j * qd_k
            QD = zeros(1,N);
            QD(j) = 1;
            QD(k) = 1;
            tau = rne(robot2, q, QD, _zeros(size(q)), [0 0 0]');
            C(:,k) = C(:,k) + (tau' - Csq(:,k) - Csq(:,j)) * qd(j);
        end
    end
    C = C + Csq * diag(qd);
endfunction 
