// Xlocate.sci   locate intersection point of xi and zi-1
// www.controlsystemslab.com   July 2012
// pxz = Xlocate(robot,A,Ti,lindex)
// Input arguments
// robot = robot data structure
// A(:,:,i) = Ai, homogeneous matrix between {i} and {i-1}
// Ti(:,:,i) = homogeneous matrix between {i} and world frame
// Ti(:,:,1) = Tb, homogeneous matrix between base and world frame
// Ti(:,:,2) = Tb*A1
// Ti(:,:,3) = Tb*A1*A2 
//  etc.
// Ti(:,:,nlinks+2)= Tb*A1*A2*...*A_nlinks*Tt;
// lindex = link index. Ex. when lindex = 1, the function returns
//          intersection point of X1 and Z0
// Notation :  {1} indicates frame 1

function pxz=Xlocate(robot,q,A,Ti,lindex)
    o_i = A(1:3,4,lindex); // origin of {i} w.r.t {i-1}
    if o_i(3)>= 0 then
        if robot.Link(lindex).RP=='R' pxz_i = [0 0 robot.Link(lindex).d 1]'; // on positive Zi
        else pxz_i = [0 0 q(lindex) 1]';
        end
    else
        if robot.Link(lindex).RP=='R' pxz_i = [0 0 -robot.Link(lindex).d 1]'; // on negative Zi
        else pxz_i = [0 0 -q(lindex) 1]';
        end            
    end
    pxz = Ti(:,:,lindex)*pxz_i;
    pxz = pxz(1:3,1);
endfunction

function pxz=xlocate(robot,q,A,Ti,lindex)
    pxz=Xlocate(robot,q,A,Ti,lindex);
endfunction