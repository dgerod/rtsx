// friction.sci joint and robot friction
// www.controlsytemslab.com  Sep 2012

// jfriction(), lfriction()  joint friction force
 // F = L.friction(QD) is the joint friction force/torque for link velocity QD.
        //
        // Notes::
        // - Friction values are referred to the motor, not the load.
        // - Viscous friction is scaled up by G^2.
        // - Coulomb friction is scaled up by G.
        // - The sign of the gear ratio is used to determine the appropriate
        //   Coulomb friction value in the non-symmetric case.


// This file is adapted from The Robotics Toolbox for Matlab (RTB).
// Copyright (C) 1993-2011, by Peter I. Corke

function tau = lfriction(l, qd)
               tau = l.B * abs(l.G) * qd;

            if qd > 0
                tau = tau + l.Tc(1);
            elseif qd < 0
                tau = tau + l.Tc(2);
            end
            tau = -abs(l.G) * tau;    // friction opposes motion 
endfunction

function tau = jfriction(l, qd)
    tau = lfriction(l, qd);
endfunction

//rfriction() Friction force of robot
//
// TAU = rfriction(robot, QD) is the vector of joint friction forces/torques for the 
// robot moving with joint velocities QD.  
//
// The friction model includes:
// - viscous friction which is linear with velocity;
// - Coulomb friction which is proportional to sign(QD).
//
function  tau = rfriction(robot, qd)

	L = robot.Link;

    tau = zeros(1,robot.nj);
	for j=1:robot.nj
		//tau(j) = L(j).friction(qd(j));
        tau(j) = lfriction(L(j),qd(j));
	end
endfunction

//nofriction() Remove friction from a robot 
//
// RNF = nofriction(robot) is a robot object with the same parameters as R but 
// with non-linear (Coulomb) friction coefficients set to zero.  
//
// RNF = nofriction(robot,'all') as above but all friction coefficients set to zero.
//
// RNF = nofriction(robot,'viscous') as above but only viscous friction coefficients 
// are set to zero.
//
// Notes::
// - Non-linear (Coulomb) friction can cause numerical problems when integrating
//   the equations of motion (R.fdyn).
// - The resulting robot object has its name string prefixed with 'NF/'.
//
// See also SerialLink.fdyn, Link.nofriction.


function  robot = nofriction(robot, varargin)

    varnum=length(varargin);  // number of arguments
    
    removeviscous = 0;
    removecoulomb = 1;
    for iv =1:varnum
        if type(varargin(iv))==10 then  // string
            varargin(iv)=convstr(varargin(iv),'l');   // convert to lower case
        end	
        if varargin(iv)=='all' then
            removeviscous = 1; 
            removecoulomb = 1;      
        elseif varargin(iv)== 'viscous' then
            removeviscous = 1;
            removecoulomb = 0;
        elseif varargin(iv)=='coulomb' than
            removecoulomb = 1;
            removeviscous = 0;
        end
    end
	for j=1:robot.nj
        if removecoulomb
            robot.Link(j).Tc = [0 0];
        end
        if removeviscous
            robot.Link(j).B = 0;
        end
    

	end
    if removeviscous & ~removecoulomb
	   robot.name = sprintf("%s (no viscous friction)",robot.name);
    elseif ~removeviscous & removecoulomb
        robot.name = sprintf("%s (no coulomb friction)",robot.name);
    elseif removeviscous & removecoulomb
        robot.name = sprintf("%s (no friction)",robot.name);     
    end
endfunction   
