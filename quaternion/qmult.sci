 // qmult.sci  multiply quaternions
 // www.controlsystemslab.com  Sep 2012
 
 function qp = qmult(q1, q2)
        //Quaternion.mtimes Multiply a quaternion object
        //
        // Q1*Q2   is a quaternion formed by the Hamilton product of two quaternions.
        // Q*V     is a vector formed by rotating the vector V by the quaternion Q.
        // Q*S     is the element-wise multiplication of quaternion elements by the scalar S.
        //
        // Notes::
        // - Overloaded operator '*'
        //
        // See also qdivide(), qpower(), qadd(), qsubtract()
        
            if isquaternion(q1) & isquaternion(q2)
            //QQMUL  Multiply unit-quaternion by unit-quaternion
            //
            //   QQ = qqmul(Q1, Q2)
            //
            //   Return a product of unit-quaternions.
            //
            //   See also: TR2Q


                // decompose into scalar and vector components
                s1 = q1.s;  v1 = q1.v;
                s2 = q2.s;  v2 = q2.v;

                // form the product
                qp = Quaternion([s1*s2-v1*v2' s1*v2+s2*v1+cross(v1,v2)']);

            elseif isquaternion(q1) & (isvec(q2)|isscalar(q2)) //isa(q1, 'Quaternion') && isa(q2, 'double')

            //QVMUL  Multiply vector by unit-quaternion
            //
            //   VT = qvmul(Q, V)
            //
            //   Rotate the vector V by the unit-quaternion Q.
            //
            //   See also: QQMUL, QINV

                if length(q2) == 3
                    qp = qmult(q1,qmult(Quaternion([0 q2(:)']),qinv(q1)));
                    //qp = q1 * Quaternion([0 q2(:)']) * qinv(q1);
                    qp = qp.v(:);
                elseif length(q2) == 1
                    
                    qp = Quaternion( q2vec(q1)*q2);
                else
                    error('quaternion-vector product: must be a 3-vector or scalar');
                end

            elseif isquaternion(q2) & (isvec(q1)|isscalar(q1)) // isa(q2, 'Quaternion') && isa(q1, 'double')
                if length(q1) == 3
                    //qp = q2 * Quaternion([0 q1(:)']) * qinv(q2);
                    qp = qmult(q2,qmult(Quaternion([0 q1(:)']),qinv(q2)));                   
                    qp = qp.v(:);
                elseif length(q1) == 1
                    qp = Quaternion( q2vec(q2)*q1);
                else
                    error('quaternion-vector product: must be a 3-vector or scalar');
                end
            end
endfunction 



