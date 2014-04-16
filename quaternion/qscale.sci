 function q = qscale(Q, r)
        //qscale( ) Interpolate rotations expressed by quaternion objects
        //
        // QI = qscale(Q, S) is a unit-quaternion that interpolates between identity for S=0
        // to Q for S=1.  This is a spherical linear interpolation (slerp) that can
        // be interpretted as interpolation along a great circle arc on a sphere.
        //
        // If S is a vector QI is a cell array of quaternions, each element
        // corresponding to sequential elements of S.
        //
        // Notes::
        // - This is a spherical linear interpolation (slerp) that can be interpretted 
        //   as interpolation along a great circle arc on a sphere.
        //
        // See also ctraj, qinterp.

   if isquaternion(Q)
            q2 = q2vec(Q);

            if min(r)<0 | max(r)>1
                error('r out of range');
            end
            q1 = [1 0 0 0];         // identity quaternion
            theta = acos(q1*q2');

            if length(r) == 1 
                if theta == 0
                    q = Q;
                else
                    q = qunit(Quaternion( (sin((1-r)*theta) * q1 + sin(r*theta) * q2) / sin(theta) ));
                end
            else
                count = 1;
                for R=r(:)'
                    if theta == 0
                        qq = Q;
                    else
                        qq = qunit(Quaternion( (sin((1-r)*theta) * q1 + sin(r*theta) * q2) / sin(theta) ));
                    end
                    q(count) = qq;
                    count = count + 1;
                end
            end
    else
        error("First argument must be a quaternion");
    end
endfunction


