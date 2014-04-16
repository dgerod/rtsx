// Xlocate4.sci   locate intersection point of xi and zi-1
// for 4-dimensional A and T matrices
// www.controlsystemslab.com   July 2012
// pxz = Xlocate(robot,A,T,j,i)
// Input arguments
// robot = robot data structure
// q = m x n matrices of joint variable sequence
// j = sequence index
// i = link index
// A(:,:,j,i) = Ai, homogeneous matrix between {i} and {i-1}
// T(:,:,j,i) = homogeneous matrix between {i} and world frame
// Ti(:,:,j,1) = Tb, homogeneous matrix between base and world frame
// Ti(:,:,j,2) = Tb*A1
// Ti(:,:,j,3) = Tb*A1*A2 
//  etc.
// Ti(:,:,j,nlinks+2)= Tb*A1*A2*...*A_nlinks*Tt;
// 
// Notation :  {1} indicates frame 1

function pxz=Xlocate4(robot,q,A,T,j,i)
    o_i = A(1:3,4,j,i); // origin of {i} w.r.t {i-1}
    if o_i(3)>= 0 then
        if robot.Link(i).RP=='R' pxz_i = [0 0 robot.Link(i).d 1]'; // on positive Zi
        else pxz_i = [0 0 q(j,i) 1]';
        end
    else
        if robot.Link(i).RP=='R' pxz_i = [0 0 -robot.Link(i).d 1]'; // on negative Zi
        else pxz_i = [0 0 -q(j,i) 1]';
        end            
    end
    pxz = T(:,:,j,i)*pxz_i;
    pxz = pxz(1:3,1);   // extract 3 x 1 vector
endfunction

function pxz=xlocate4(robot,q,A,T,j,i)
    pxz=Xlocate4(robot,q,A,T,j,i);
endfunction