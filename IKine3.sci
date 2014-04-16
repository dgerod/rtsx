// ikine3.sci   Inverse kinematics for 3-axis robot with no wrist
// www.controlsystemslab.com   August 2012
// This function is adapted from the Robotic Toolbox for MATLAB, Copyright (C) 
// 1993-2011, by Peter I. Corke


// q = IKine3(robot, T) is the joint coordinates corresponding to the robot
// end-effector pose T represented by the homogenenous transform.  This
// is a analytic solution for a 3-axis robot (such as the first three joints
// of a robot like the Puma 560).
//
// q= IKine6s(robot, T, CONFIG) as above but specifies the configuration of the arm in
// the form of a string containing one or more of the configuration codes:
//
// 'l'   arm to the left (default)
// 'r'   arm to the right
// 'u'   elbow up (default)
// 'd'   elbow down

//
// Notes::
// - The same as ikine6s without the wrist.
// - The inverse kinematic solution is generally not unique, and 
//   depends on the configuration string.
// - Joint offsets, if defined, are added to the inverse kinematics to 
//   generate q.
//
function q = IKine3(robot, T, varargin)
    q = _ikine3(robot, T, varargin);    
endfunction

function q = ikine3(robot, T, varargin)
    q = _ikine3(robot, T, varargin);    
endfunction

function q = _ikine3(robot, T, varargin)
    varargin = varargin($);
    if strcmp(robot.conf, 'RRR')
        error('Solution only applicable for 3DOF all-revolute manipulator');
    end

    if robot.mdh ~= 0
        error('Solution only applicable for standard DH conventions');
    end

    if ndims(T) == 3
        theta = zeros(size(T,3),robot.nj);
        for k=1:size(T,3)
            theta(k,:) = _ikine6s(robot, T(:,:,k), varargin);
        end
        q = theta;
        return;
    end
    L = robot.Link;
    a2 = L(2).a;
    a3 = L(3).a;

    if ~isspherical(robot)
        error('wrist is not spherical')
    end

    d3 = L(3).d;

    if ~ishomog(T)
        error('T is not a homog xform');
    end

    // undo base transformation
    T = robot.base\T;
    
    // The following parameters are extracted from the Homogeneous 
    // Transformation as defined in equation 1, p. 34


    Px = T(1,4);
    Py = T(2,4);
    Pz = T(3,4);

    // The configuration parameter determines what n1,n2,n4 values are used
    // and how many solutions are determined which have values of -1 or +1.

    //nargin = argn(2);
    //pause;
    if length(varargin)== 0,
        configuration = '';
    else
        configuration = convstr(varargin(1),'l');

    end

    // default configuration

    n1 = -1;    // L
    n2 = -1;    // U


    if strindex(configuration,'l')~=[] //~isempty(strfind(configuration, 'l'))
        n1 = -1;
    end
    if strindex(configuration,'r')~=[] //~isempty(strfind(configuration, 'r'))
        n1 = 1;
    end
    if strindex(configuration,'u')~=[] //~isempty(strfind(configuration, 'u'))
        if n1 == 1
            n2 = 1;
        else
            n2 = -1;
        end
    end
    if strindex(configuration,'d')~=[] //~isempty(strfind(configuration, 'd'))
        if n1 == 1
            n2 = -1;
        else
            n2 = 1;
        end
    end



    //
    // Solve for theta(1)
    // 
    // r is defined in equation 38, p. 39.
    // theta(1) uses equations 40 and 41, p.39, 
    // based on the configuration parameter n1
    //

    r=sqrt(Px^2 + Py^2);
    if (n1 == 1)
        theta(1)= atan(Py,Px) + asin(d3/r);
    else
        theta(1)= atan(Py,Px) + pi - asin(d3/r);
    end

    //
    // Solve for theta(2)
    //
    // V114 is defined in equation 43, p.39.
    // r is defined in equation 47, p.39.
    // Psi is defined in equation 49, p.40.
    // theta(2) uses equations 50 and 51, p.40, based on the configuration 
    // parameter n2
    //

    V114= Px*cos(theta(1)) + Py*sin(theta(1));
    r=sqrt(V114^2 + Pz^2);
    Psi = acos((a2^2-d4^2-a3^2+V114^2+Pz^2)/(2.0*a2*r));
    if ~isreal(Psi)
        wmsg = sprintf("%s -- point not reachable",robot.name);
        warning(wmsg);
        theta = [%nan %nan %nan %nan %nan %nan];
        q = theta;
        return
    end
    theta(2) = atan(Pz,V114) + n2*Psi;

    //
    // Solve for theta(3)
    //
    // theta(3) uses equation 57, p. 40.
    //

    num = cos(theta(2))*V114+sin(theta(2))*Pz-a2;
    den = cos(theta(2))*Pz - sin(theta(2))*V114;
    theta(3) = atan(a3,d4) - atan(num, den);


    // remove the link offset angles
    for i=1:robot.nj   //#ok<*AGROW>
        theta(i) = theta(i) - L(i).offset;
    end
    q=clean(theta');
    
endfunction
