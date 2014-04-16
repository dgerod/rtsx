// linecenter.sci  find center of a line in 3-D
// www.controlsystemslab.com   July 2012
// c = linecenter(p1,p2) where pi, p2 are 3x1 vector representing points
// at the end of line

function c=linecenter(p1,p2)
    c = p1 + 0.5*(p2 - p1);
endfunction