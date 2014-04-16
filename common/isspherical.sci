// isspherical.sci   Test for a spherical wrist
// www.controlsystemslab.com   August 2012
// R.isspherical() is true if the robot has a spherical wrist, that is, the 
// last 3 axes are revolute and their axes intersect at a point.

function v = isspherical(robot) 
    nj = robot.nj;
    L = robot.Link(nj-2:nj);
    v = 0;
    // first test that all lengths are zero
    if and([L(1).a, L(2).a, L(3).a, L(2).d, L(3).d])
        return;
    end
    if (abs(L(1).alpha) == pi/2) & (abs(L(1).alpha + L(2).alpha)< %eps)
        v = 1;
    end
endfunction