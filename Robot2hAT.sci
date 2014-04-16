// Robot2hATs.sci generates  4-dimensional matrices "A" and "T" matrices 
// from robot information and rows of joint variables
// www.controlsystemslab.com    July 2012
//
// Usage: [A,T] = Robot2hAT(robot,q); where q is m x n matrices containing m sets of
// n joint variables
// 
// if L contains n links, A becomes a 4 x 4 x m x n matrix
// and T is the overall homogeneous transformation; i.e, T=A1*...*An
//Notes:
// - For a revolute joint the THETA parameter of the link is ignored, and Q used instead.
// - For a prismatic joint the D parameter of the link is ignored, and Q used instead.
// - The link offset parameter is added to Q before computation of the transformation matrix.


function [A,T]=Robot2hAT(robot,q)
      sdhflag = 0;        // flag for stdDH
      mdhflag = 0;        // flag for modDH
      L=robot.Link;
      
      nlinks=size(L,1);    // number of links 
      nseqs = size(q,1);    // number of sequences
      if nlinks~=size(q,2) then
          error("Numbers of links and joint variables do not match.")
      end
      A=hypermat([4 4 nseqs nlinks+2]); // generate hypermatrices
      for i=1:nseqs
          A(:,:,i,1)=robot.base;
          A(:,:,i,nlinks+2)=robot.tool;
      end
      T=A;
      for j = 1:nseqs
          for i = 1:nlinks
              sa = sin(L(i).alpha); ca = cos(L(i).alpha);
              qt = q(j,i) + L(i).offset;
              if L(i).sigma == 0 then //revolute
                  st = sin(qt); ct = cos(qt);
                  d = L(i).d;
              else    // prismatic
                  st = sin(L(i).theta); ct = cos(L(i).theta);
                  d = qt;
              end
              if L(i).mdh == 0 // standard DH 
                  sdhflag = 1;    // set flag for standard DH
                  A(:,:,j,i+1) = clean([ct -st*ca st*sa L(i).a*ct;
                              st ct*ca  -ct*sa L(i).a*st;
                              0   sa       ca      d;
                              0    0        0       1]);
              else // modified DH
                  mdhflag = 1;    // set flag for modified DH
                  A(:,:,j,i+1) = clean([ct   -st       0    L(i).a;
                              st*ca ct*ca   -sa    -sa*d;
                              st*sa  ct*sa   ca      ca*d;
                              0       0        0       1]);               
              end
              T(:,:,j,i+1) = clean(T(:,:,j,i)*A(:,:,j,i+1));    // compute T matrix
           end
           T(:,:,j,i+2)=clean(T(:,:,j,i+1)*A(:,:,j,i+2)); // including tool frame
       end
       if (sdhflag & mdhflag) // inconsistency in link structure
       // it does not make sense to compute T matrix
              sprintf("Links contain both standard and modified DH!\n");
              sprintf("T matrix is not computed\n");
              A = [];
              T = [];
        end 
endfunction

function [A,T]=robot2hat(robot,q)
    [A,T]=Robot2hAT(robot,q);
endfunction