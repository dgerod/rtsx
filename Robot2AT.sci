// Robot2AT.sci find A and T matrices from a robot model
// www.controlsystemslab.com   Nov 2012
// Robot2AT simply calls Link2AT
// Options are 
//  'all'  include base and tool transform
//  'base' include base 
//  'tool' include tool
//  'none' no base/tool (default)
// only homogeneous matrix T affected by options. A remains unchanged

function [A,T,Tb,Tt]=Robot2AT(robot,q,varargin)
    [A,T,Tb,Tt]=_Robot_2_AT(robot,q,varargin)
endfunction

function [A,T,Tb,Tt]=robot2at(robot,q,varargin)
    [A,T,Tb,Tt]=_Robot_2_AT(robot,q,varargin);
endfunction

function [A,T,Tb,Tt]=_Robot_2_AT(robot,q,varargin)
    varargin=varargin($);
    varnum=length(varargin);  // number of arguments
    atchoice = 0;   // 0 = none, 1=w/base, 2=w/tool, 3 = all
    for iv =1:varnum
        if type(varargin(iv))==10 then  // string
            varargin(iv)=convstr(varargin(iv),'l');   // convert to lower case
        end
        if varargin(iv)=='none' then
            atchoice=0;       
        elseif varargin(iv)== 'base' then
            if atchoice==0 then
                atchoice = 1;
            elseif atchoice==2 then // if tool previously selected, change mode to 'all'
                atchoice = 3; 
            end
        elseif varargin(iv)== 'tool' then
            if atchoice==0 then
                atchoice = 2;
            elseif atchoice==1 then // if base previously selected, change mode to 'all'
                atchoice = 3; 
            end
        elseif varargin(iv)=='all' then
            atchoice=3;     
        end
    end    
    
    
    L = robot.Link;
    [A,T]=Link2AT(L,q);
    Tb=robot.base;
    Tt=robot.tool;
    select atchoice
    case 1
        T=robot.base*T;
    case 2        
        T=T*robot.tool;
    case 3
        T=robot.base*T*robot.tool;
    end
    T=clean(T);
endfunction
