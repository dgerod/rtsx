// qinterp.sci  
function q = qinterp(Q1, Q2, r)
        //qinterp() Interpolate quaternions
        //
        // QI = qinterp(Q2, S) is a unit-quaternion that interpolates a rotation 
        // between Q1 for S=0 and Q2 for S=1.
        //
        // If S is a vector QI is a vector of quaternions, each element
        // corresponding to sequential elements of S.
        //
        // Notes::
        // - This is a spherical linear interpolation (slerp) that can be interpretted 
        //   as interpolation along a great circle arc on a sphere.
        // - The value of S is clipped to the interval 0 to 1.
        //
        // See also ctraj, Quaternion.scale.
   if isquaternion(Q1) & isquaternion(Q2)
            q1 = q2vec(Q1);
            q2 = q2vec(Q2);

            theta = acos(q1*q2');
            count = 1;

            // clip values of r
            r(r<0) = 0;
            r(r>1) = 1;
            
            if length(r) == 1
                if theta == 0
                    q = Q1;
                else
                    q = Quaternion( (sin((1-r)*theta) * q1 + sin(r*theta) * q2) / sin(theta) );
                end
            else
                for R=r(:)'
                    if theta == 0
                        qq = Q1;
                    else
                        qq = Quaternion( (sin((1-R)*theta) * q1 + sin(R*theta) * q2) / sin(theta) );
                    end
                    q(count) = qq;
                    count = count + 1;
                end
            end
    else
        error("Arguments must be quaternions");
    end // if isquaternion(Q1) & isquaternion(Q2)
endfunction
        

