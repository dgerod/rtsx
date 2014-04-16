// Link2AT.sci generates "A" and "T" matrix from link information
// www.controlsystemslab.com    July 2012
//
// Usage: [A,T] = Link2AT(L,q);
// if L contains n links, A becomes a 4 x 4 x n matrix
// and T is the overall homogeneous transformation; i.e, T=A1*...*An
//Notes:
// - For a revolute joint the THETA parameter of the link is ignored, and Q used instead.
// - For a prismatic joint the D parameter of the link is ignored, and Q used instead.
// - The link offset parameter is added to Q before computation of the transformation matrix.


function [A,T,Ti]=Link2AT(L,q)
     sdhflag = 0;        // flag for stdDH
     mdhflag = 0;        // flag for modDH
     T = eye(4,4);     // initial T matrix
      nlinks=size(L,1);    // number of links 
      if nlinks~=length(q) then
          error("Numbers of links and joint variables do not match.")
      end
      for i = 1:nlinks
          sa = sin(L(i).alpha); ca = cos(L(i).alpha);
          qt = q(i) + L(i).offset;
          if L(i).sigma == 0 then //revolute
              st = sin(qt); ct = cos(qt);
              d = L(i).d;
          else    // prismatic
              st = sin(L(i).theta); ct = cos(L(i).theta);
              d = qt;
          end
          if L(i).mdh == 0 // standard DH 
              sdhflag = 1;    // set flag for standard DH
              A(:,:,i) = clean([ct -st*ca st*sa L(i).a*ct;
                          st ct*ca  -ct*sa L(i).a*st;
                          0   sa       ca      d;
                          0    0        0       1]);
          else // modified DH
              mdhflag = 1;    // set flag for modified DH
              A(:,:,i) = clean([ct   -st       0    L(i).a;
                          st*ca ct*ca   -sa    -sa*d;
                          st*sa  ct*sa   ca      ca*d;
                          0       0        0       1]);               
          end
          T = clean(T*A(:,:,i));    // compute T matrix
          Ti(:,:,i)=T;
       end
       if (sdhflag & mdhflag) // inconsistency in link structure
       // it does not make sense to compute T matrix
              sprintf("Links contain both standard and modified DH!\n");
              sprintf("T matrix is not computed\n");
              T = [];
        end
        T=T; 
endfunction

function [A,T,Ti]=link2at(L,q)
    [A,T,Ti]=Link2AT(L,q);
endfunction

