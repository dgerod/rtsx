// linesmap.sci  map  line style string to Scilab line_style number
// www.controlsystemslab.com   August 2012

function lsnum = linesmap(lstyle)
    if argn(2)==0
        printf("Line style choices are : solid (-), dash (--), dash dot (-.), longdash dot (--.), bigdash dot (=.), bigdash longdash (=--, dot (.), double dot (..) (must put color in quotes)\n");
    else
        if type(lstyle)~=10  // not a string
            error("linesmap: must input a string argument");
         end
        select lstyle,
        case 'solid' then lsnum = 1,
        case '-' then lsnum = 1,
        case 'dash' then lsnum = 2,
        case '--' then lsnum = 2,
        case 'dash dot' then lsnum = 3,
        case '-.' then lsnum = 3,
        case 'longdash dot' then lsnum = 4,
        case '--.' then lsnum = 4,
        case 'bigdash dot' then lsnum = 5,
        case '=.' then lsnum = 5,
        case 'bigdash longdash' then lsnum = 6,
        case '=--' then lsnum = 6,
        case 'dot' then lsnum = 7,
        case '.' then lsnum = 7,
        case 'double dot' then lsnum = 8,
        case '..' then lsnum = 8,
        else
            printf("linesmap: %s -- invalid line style. Defaulted to solid\n",lstyle);
            lsnum = 1;
        end
    end
endfunction

