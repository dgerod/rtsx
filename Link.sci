// Link.sci  create robot manipulator link
// www.controlsystemslab.com  July 2012
// 
// Link function returns a data structrue that holds all information related
// to a robot link such as kinematics parameters, rigid-body inertial 
// parameters, motor and transmission parameters.
//
//  theta    kinematic: joint angle
//  d        kinematic: link offset
//  a        kinematic: link length
//  alpha    kinematic: link twist
//  sigma    kinematic: 0 if revolute, 1 if prismatic
//  RP       kinematic: 'R' if revolute, 'P' if prismatic (for easy display)
//  mdh      kinematic: 0 if standard D&H, else 1
//  offset   kinematic: joint variable offset
//  qlim     kinematic: joint variable limits [min max]
//
//  m        dynamic: link mass
//  r        dynamic: link COG wrt link coordinate frame 3x1
//  I        dynamic: link inertia matrix, symmetric 3x3, about link COG.
//  B        dynamic: link viscous friction (motor referred)
//  Tc       dynamic: link Coulomb friction
//
//  G        actuator: gear ratio
//  Jm       actuator: motor inertia (motor referred)
//
// Usage:  L = Link([theta d a alpha],joint_type, options)
//Example:
// L(1)=Link([0 0 1 0],'R');
// L(2)=Link([0 1 0 0],'P');

// This function is a part of Robotic Tools for Scilab/Xcos (RTSX)
//
// inspired by The Robotics Toolbox for Matlab (RTB).Copyright (C) 1993-2011,
// by Peter I. Corke
// Note: There are some differences due to the current version of Scilab 
// does not support OOP. 

function L=Link(lparam,jtype,varargin)
    narg = argn(2);   // number of input arguments
    if narg==0 
       error("Usage:  L = Link([theta d a alpha <offset>],<joint_type>,<DH_type>,<optional parameter, value>)");
       
    end
    if length(lparam) < 4
        error('must provide params [theta d a alpha]');
    end

    L.theta = lparam(1);
    L.d = lparam(2);
    L.a = lparam(3);
    L.alpha = lparam(4);

    L.RP = 'R';  // default to revolute
    L.sigma = 0; // 0= R , 1=P. This is redundant to RP
    L.mdh = 0;  // default to standard DH 
    L.offset = 0;
    L.qlim = [-1e6 1e6];  // default to virtually no limit
        if length(lparam)>4 then L.offset = lparam(5) 
    end
    if narg>1 then    
        if jtype==1 | jtype=='P'|jtype=='p' then
            L.RP='P';
            L.sigma = 1;
        else 
            L.RP='R';
            L.sigma = 0; 
        end
    end
//    if narg>2 then
//        if dhtype == 1 | dhtype =='mdh' | dhtype =='mDH' then
//            L.mdh = 1;    // modified DH
//        else
//            L.mdh = 0;  // standard DH default
//        end
//    end
    
    L.m = [];
    L.r = [];
    L.I = [];
    L.Jm = [];
    L.G = [];
    L.B = 0;
    L.Tc = [0 0];
    if narg>2 then
        varnum=length(varargin);  // number of variable arguments
        
        if pmodulo(varnum,2) then  // number of arguments is odd. Error!
            error("Input argument number is odd. You must miss something!");
        else
            for iv =1:2:varnum-1  // select only command at odd position
                if type(varargin(iv)==10) then  // check if string (it should be!)
                    varargin(iv)=convstr(varargin(iv),'l'); // convert to lower 
                end
                select varargin(iv),
                case 'dhtype' then
                        parm = varargin(iv+1);
                        if type(parm)==1    // number
                            if parm == 1    // modified DH
                                L.mdh = 1;
                            else
                                L.mdh = 0;    // standard DH
                            end
                        elseif type(parm)==10  // string
                            parm=convstr(parm,'l');
                            if parm == 'mdh'
                                L.mdh = 1;
                            else
                                L.mdh = 0;
                            end
                         end
                    
                case 'qlim' then   // joint limits
                        qlim = varargin(iv+1);
                        if size(qlim)==[1 2] 
                            L.qlim = qlim;
                        else error("Joint limits must be a 1x2 vector");
                        end

                    case 'dynparm' then  // dynamic parameters
                        parm = varargin(iv+1);
                        if size(parm)==[1 15] 
                            L.r = parm(1,1:3);
                            v = parm(1,4:9);
                            L.I = [v(1),v(4),v(6);v(4),v(2),v(5);v(6),v(5),v(3)];
                            L.m = parm(1,10);
                            L.Jm = parm(1,11);
                            L.G = parm(1,12);
                            L.B = parm(1,13);
                            L.Tc = parm(1,14:15);
                         else error("Dynamic parameters must be 1x15 vector");
                         end  
                    case 'r' then 
                        parm = varargin(iv+1);
                        if type(parm)== 1 & size(parm)==[1 3] // 1x3 vector
                            L.r = parm;
                        else error("r value must be a 1x3 vector");
                        end                                                                                                    case 'i' then 
                        parm = varargin(iv+1);
                        if type(parm)== 1 & size(parm)==[1 6] // a 1x6 vector
                            v = parm;
                            L.I = [v(1),v(4),v(6);v(4),v(2),v(5);v(6),v(5),v(3)];
                        else error("I value must be a 1x6 vector");
                        end 
                    case 'm' then 
                        parm = varargin(iv+1);
                        if type(parm)== 1 & size(parm)==[1 1] // a scalar
                            L.m = parm;
                        else error("m value must be a number");
                        end                                                                                                 case 'jm' then 
                        parm = varargin(iv+1);
                        if type(parm)== 1 & size(parm)==[1 1] // a scalar
                            L.Jm = parm;
                        else error("Jm value must be a number");
                        end                                                                                                  case 'g' then 
                        parm = varargin(iv+1);
                        if type(parm)== 1 & size(parm)==[1 1] // a scalar
                            L.G = parm;
                        else error("G value must be a number");
                        end                                                                                                 case 'b' then 
                        parm = varargin(iv+1);
                        if type(parm)== 1 & size(parm)==[1 1] // a scalar
                            L.B = parm;
                        else error("B value must be a number");
                        end                                                                                                 case 'tc' then 
                        parm = varargin(iv+1);
                        if type(parm)== 1 & size(parm)==[1 2] // a 1x2 vector
                            L.Tc = parm;
                        else error("Tc value must be a 1x2 vector");
                        end                                                                                                                                              
                    else
                        emsg= sprintf("Invalid link parameter %s",varargin(iv)); 
                        error(emsg); 
                end // select varargin(iv)
            end  // for iv=1:varnum
        end    // if pmodulo(varnum,2)     
    end  // if narg>3
endfunction
