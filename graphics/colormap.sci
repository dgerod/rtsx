// colormap.sci  map color string to Scilab color number
// www.controlsystemslab.com   August 2012

function cnum = colormap(colorstr)
   if argn(2)==0
        printf("Color choices are -- black, blue, green, cyan, red, purple, yellow, white (must put color in quotes)\n");
    else    
        if type(colorstr)~=10  // not a string
            error("colormap: must input a string argument");
         end
        select colorstr,
            case 'black' then cnum = 1,
            case 'k' then cnum = 1, 
            case 'blue' then cnum = 2,
            case 'b' then cnum = 2,
            case 'green' then cnum = 3,
            case 'g' then cnum = 3,
            case 'cyan' then cnum = 4,
            case 'c' then cnum = 4,
            case 'red' then cnum = 5,
            case 'r' then cnum = 5,
            case 'purple' then cnum = 6,
            case 'magenta' then cnum = 6,
            case 'm' then cnum = 6,
            case 'yellow' then cnum = 7,
            case 'y' then cnum = 7,
            case 'white' then cnum = 8,
            case 'w' then cnum = 8,
            else
                printf("colormap: %s -- invalid color. Defaulted to black\n",colorstr);
                cnum = 1;
       end
     end        
endfunction

