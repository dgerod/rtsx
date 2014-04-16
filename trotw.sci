// trotw.sci rotation of a coordinate frame about world axis
// origin must be at [0,0,0]'
// www.controlsystemslab.com   August 2012
// usage: R = trotw(R,theta,'x','deg');
//        T = trotw(T,theta,'y');

function Tr=trotw(T,theta,varargin)
    rflag = 0;
    if isrot(T,'valid') then    // a 3x3 rotation matrix
        T=r2t(T);
        rflag = 1;
    elseif ishomog(T,'valid') then
        if ~isequal(T(1:3,4),[0 0 0]') then
            error("Frame origin must be at [0 0 0]");
        end
    else
        error("Invalid 1st input argument");    
    end
    opt.deg = 0;
    
    // process varargin 
    numopts=length(varargin); // find number of options
    Xrot = 0; Yrot = 0; Zrot = 0; // rotation axis flag
    if numopts>0 then
        for i=1:numopts
            if varargin(i)== 'deg' then 
                opt.deg = 1;
            elseif varargin(i)=='x'|varargin(i)=='X'
                Xrot = 1;
            elseif varargin(i)=='y'|varargin(i)=='Y'
                Yrot = 1;
            elseif varargin(i)=='z'|varargin(i)=='Z'
                Zrot = 1;                
            end
        end
    end
    // optionally convert from degrees
    if opt.deg then
        d2r = %pi/180.0;
        theta = d2r*theta;
    end   
    rpy = tr2rpy(T);
    if Xrot then    
        Tr = trotx(theta)*trotx(rpy(1))*troty(rpy(2))*trotz(rpy(3));
    elseif Yrot then
        Tr = troty(theta)*trotx(rpy(1))*troty(rpy(2))*trotz(rpy(3)); 
    elseif Zrot then
        Tr = trotz(theta)*trotx(rpy(1))*troty(rpy(2))*trotz(rpy(3));
    else
        printf("\nNo rotation axis given. Frame is not rotated.\n");
        Tr = T;
     end   
     if rflag
         Tr = Tr(1:3,1:3);
     end    
         
endfunction