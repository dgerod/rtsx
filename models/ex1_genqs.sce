// ex1_genqs.sce  generates qs for robot in example 1
// for testing purpose
// www.controlsystemslab.com   July 2012

t = [0:0.01:1]';        // Time data
qs = [sin(2*pi*t) 1.5+sin(2*pi*t) sin(2*pi*t)-pi/2];

