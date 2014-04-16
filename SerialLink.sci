// SerialLink.sci creates a SerialLink robot structure
// www.controlsystemslab.com  July 2012
//
// Parameters
// Link(i)      Link structure (inherited from L)
//  gravity    direction of gravity [gx gy gz]
//  base       pose of robot's base (4x4 homog xform)
//  tool       robot's tool transform, T6 to tool tip (4x4 homog xform)
//  name       name of robot, used for graphical display
//  manuf      annotation, manufacturer's name
//
//  nj           number of joints
//  config      joint configuration string, eg. 'RRRRRR'
//  mdh         kinematic convention boolean (0=DH, 1=MDH)
//
// Usage: RM = SerialLink(L, options)

function rm=seriallink(L,varargin)
    rm=_Serial_Link(L,varargin);
endfunction

function rm=SerialLink(L,varargin)
    rm=_Serial_Link(L,varargin);
endfunction

function rm=_Serial_Link(L,varargin)
    varargin = varargin($);
     varnum=length(varargin);
     sdhflag = 0;        // flag for stdDH
     mdhflag = 0;        // flag for modDH
     nlinks=size(L,1);    // number of links
     rm.nj = nlinks;      // number of joints
     rm.mdh = 0;         // default to stdDH
     rm.conf = '';        // joint configuration string
     for i=1:nlinks
         rm.Link(i)=L(i);
         if rm.Link(i).mdh then 
             mdhflag = 1;  // modified DH detected
             rm.mdh = 1;    // modDH 
         else sdhflag = 1; // standard DH detected
         end
         if L(i).sigma then rm.conf=strcat([rm.conf,'P']);
         else rm.conf=strcat([rm.conf,'R']);
         end
     end
     if (sdhflag & mdhflag) then // mixed DH conventions not allowed
         rm = []; //return null matrix
         error('Robot has mixed DH link conventions');
     end
     rm.name = '';
     rm.manuf = '';
     rm.comment = '';
     rm.base = eye(4,4);
     rm.tool = eye(4,4);
     rm.gravity = [0; 0; 9.81];
     rm.workspace=[];
     rm.viewangle=[];
     
     if pmodulo(varnum,2) then  // number of arguments is odd. Error!
          error("Input argument number is odd. You must miss something!");
     else
        for iv =1:2:varnum-1  // select only command at odd position
            if type(varargin(iv))==10 then  // check if string (it should be!)
                varargin(iv)=convstr(varargin(iv),'l'); // convert to lower 
            else
                error("Parameter name must be a string (perhaps you forget to put it in quotes)!");                
            end
            select varargin(iv),
                case 'name' then 
                    if  type(varargin(iv+1))==10 then
                         rm.name = varargin(iv+1);
                    else error("Robot name must be a string");
                    end
                case 'manuf' then
                    if  type(varargin(iv+1))==10 then
                        rm.manuf = varargin(iv+1);
                    else error("Robot manufacturer must be a string");
                    end
                case 'comment' then
                    if  type(varargin(iv+1))==10 then
                        rm.comment = varargin(iv+1);
                    else error("Comment must be a string");
                    end                    
                case 'viewangle' then
                    if size(varargin(iv+1))==[1,2] then                       
                        rm.viewangle = varargin(iv+1);
                    else error("Value for robot viewangle must be 1 x 2 vector");
                    end
                case 'base' then
                    if ishomog(varargin(iv+1),'valid') then
                        rm.base = varargin(iv+1);
                    else error("Value for robot base must be 4 x 4 matrix");
                    end
                case 'tool' then
                    if ishomog(varargin(iv+1),'valid') then
                        rm.tool = varargin(iv+1);
                    else error("Value for robot tool must be 4 x 4 matrix");
                    end
                case 'gravity' then
                    if size(varargin(iv+1))== [3 1] then
                        rm.gravity = varargin(iv+1);
                    else error("Value for robot gravity must be 3 x 1 vector");
                    end
                end // select varargin(iv)
            end // for iv=1:2:varnum-1
        end // if pmodulo(varnum,2)
endfunction

