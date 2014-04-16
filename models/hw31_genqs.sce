// hw31_genqs.sce  generates qs for robot in Homework 3 problem 1
// for testing purpose
// www.controlsystemslab.com   July 2012

t = [0:0.01:1]';        // Time data
qs = [3*pi/4+sin(2*pi*t) 1.5+sin(2*pi*t) pi/4*sin(2*pi*t)];

