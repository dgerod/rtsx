// rne.sci computes inverse dynamics using Newton-Euler algorithm
// www.controlsystemslab.com  Sep 2012

// TAU = rne(robot, Q, QD, QDD) is the joint torque required for the robot R
// to achieve the specified joint position Q, velocity QD and acceleration QDD.
//
// TAU = rne(robot, Q, QD, QDD, GRAV) as above but overriding the gravitational 
// acceleration vector in the robot object R.
//
// TAU = rne(robot, Q, QD, QDD, GRAV, FEXT) as above but specifying a wrench 
// acting on the end of the manipulator which is a 6-vector [Fx Fy Fz Mx My Mz].
//
// TAU = rne(robot, X) as above where X=[Q,QD,QDD].
//
// TAU = rne(robot, X, GRAV) as above but overriding the gravitational 
// acceleration vector in the robot object R.
//
// TAU = rne(robot, X, GRAV, FEXT) as above but specifying a wrench 
// acting on the end of the manipulator which is a 6-vector [Fx Fy Fz Mx My Mz].
//
// [TAU,WBASE] = rne(robot, X, GRAV, FEXT) as above but the extra output is the
// wrench on the base.
//
// If Q,QD and QDD (MxN), or X (Mx3N) are matrices with M rows representing a 
// trajectory then TAU (MxN) is a matrix with rows corresponding to each trajectory 
// step.
//
// Notes::
// - The robot base transform is ignored.
// - The torque computed contains a contribution due to armature
//   inertia and joint friction.

// - The function is a wrapper which calls either RNE_DH or RNE_MDH depending on 
//   the kinematic conventions used by the robot object.

//
// See also SerialLink.accel, SerialLink.gravload, SerialLink.inertia.


//
// This file is adapted from The Robotics Toolbox for Matlab (RTB).
// Copyright (C) 1993-2011, by Peter I. Corke



function [tau,wbase] = rne(robot, varargin)

    if robot.mdh == 0
        [tau,wbase] = rne_dh(robot, varargin(:));
    else
        printf("At this phase only DH type is implemented");
        //[varargout{1:nargout}] = rne_mdh(robot, varargin{:});
    end
endfunction
