function robot = payload(robot, m, p)
 //Add payload mass
 //
 // payload(robot, M, P) adds a payload with point mass M at position P 
 // in the end-effector coordinate frame.
 //
 // See also rne(), gravload().
     if type(m) ~= 1 | length(m) ~= 1
         error("Bad m argument");
     end
     if isvec(p)
         p = p(:)';
     else
         error("Bad p argument");
     end
     n = robot.nj;
     robot.Link(n).m = m;
     robot.Link(n).r = p;

endfunction
