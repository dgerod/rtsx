// =====================================================================
// Frame.sci   make a coordinate frame
// www.controlsystemslab.com  August 2012
// =====================================================================

function [F] = _frame (T,varargin)
// usage: F = Frame(T,)
// Make a coordinate frame
//
    
    AbsFlag = 0;  // defaulted to relative 
    F.Tabs = [];    // absolute frame description T_n,0
    F.Trel = [];    // relative frame description T_n,n-1
    F.name = '';  
    F.completed = 0; // completion flag  
    
    if T ~= [] then
    
        // check if T is a valid homogeneous transformation
        if ~(ishomog(T,'valid')) then
            error("T is not a valid homogeneous transformation");
        end
    end    
    
    varargin=varargin($);
    varnum=length(varargin);

    for iv =1:varnum  // find all variable arguments
        
        if type(varargin(iv)==10) then  // check if string (it should be!)
            varargin(iv)=convstr(varargin(iv),'l'); // convert to lower 
        else
            error("Frame() accepts only string parameters (perhaps you forget to put it in quotes)!");              
        end
        
        select varargin(iv),
            case 'rel' then 
                 AbsFlag = 0;
            case 'abs' then
                 AbsFlag = 1;
            case 'name' then
                    if  type(varargin(iv+1))==10 then
                         F.name = varargin(iv+1);
                         break;
                    else error("Frame name must be a string");
                    end                 
            else error("Invalid input parameter");
         end
    end
    
    if AbsFlag then
        F.Tabs=T;
    else 
        F.Trel=T;
    end
    
endfunction

// ---------------------------------------------------------------------

Frame =_frame;
frame = _frame;

// =====================================================================
