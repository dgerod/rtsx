// gentrot.sci  generate T for basic rotations

function T = gentrot(axis,th,varargin)
    opt.deg = 0;
    opt.nsteps = 50;
    // process varargin 
    varnum=length(varargin);  // number of arguments
    for iv =1:varnum
 
        if varargin(iv)=='deg' then
            opt.deg = 1;

        elseif varargin(iv)=='nsteps' & (iv<varnum) then
            opt.nsteps = varargin(iv);
  
        end  
    end 
    if opt.deg
        d2r = pi/180;
        th = d2r*th;
     end   
     vth = 0:th/opt.nsteps:th;
     lvth = length(vth);
    if type(axis)~= 10
        error("Rotation axis must be a character in quotes");
    else
        axis = convstr(axis,'l');
        select axis
        case 'x'
            for i=1:lvth
                T(:,:,i) = trotx(vth(i));
             end
        case 'y'
            for i=1:lvth
                T(:,:,i) = troty(vth(i));
             end            
        case 'z'
            for i=1:lvth
                T(:,:,i) = trotz(vth(i));
             end    
       else
           error("Invalid axis label. Only x, y, z accepted");
       end
   end
    
endfunction     
