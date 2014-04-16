//MKGRID Create grid of points
//
// P = MKGRID(N, S, OPTIONS) is a set of points (3 x N^2) that define a NxN planar
// grid of points with side length S.  The points are the columns of P.
// If N is a 2-vector the grid is N(1)xN(2) points.  If S is a 2-vector the 
// side lengths are S(1)xS(2).
//
// By default the grid lies in the XY plane, symmetric about the origin.
//
// Options::
// 'T',T   the homogeneous transform T is applied to all points, allowing the 
//         plane to be translated or rotated.


// This file is adapted from The Machine Vision Toolbox for Matlab (MVTB).
// Copyright (C) 1993-2011, by Peter I. Corke
//


function p = mkgrid(N, s, varargin)
    
    
    
    opt.T = [];
    
    varnum=length(varargin);
        for iv =1:varnum  // select only command at odd position
         if type(varargin(iv))==10 then  // check if string 
              varargin(iv)=convstr(varargin(iv),'l'); // convert to lower 
          end
          if type(varargin(iv))==10 then
          
              select varargin(iv),
                   case 't' then
                     if ishomog(varargin(iv+1))
                         opt.T = varargin(iv+1);
                     else error('Wrong data type for T. Must be a 4 x 4 homogeneous matrix');
                     end
                 end // select varargin(iv)
          end // if type(varargin(iv)==10)
      end // for iv=1:varnum-1


    if length(s) == 1,
        sx = s; sy = s;
    else
        sx = s(1); sy = s(2);
    end

    if length(N) == 1,
        nx = N; ny = N;
    else
        nx = N(1); ny = N(2);
    end


    if N == 2,
        // special case, we want the points in specific order
        p = [-sx -sy 0
             -sx  sy 0
              sx  sy 0
              sx -sy 0]'/2;
    else
        [X, Y] = meshgrid(1:nx, 1:ny);
        X = ( (X-1) / (nx-1) - 0.5 ) * sx;
        Y = ( (Y-1) / (ny-1) - 0.5 ) * sy;
        Z = _zeros(size(X));
        p = [X(:) Y(:) Z(:)]';
    end
    
    // optionally transform the points
    if ~isempty(opt.T)
        p = homtrans(opt.T, p);
    end
endfunction
