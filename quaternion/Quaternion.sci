// Quaternion.sci   create a quaternion data structure
// www.controlsystemslab.com   Sep 2012
// A quaternion is a compact method of representing a 3D rotation that has
// computational advantages including speed and numerical robustness.
// A quaternion has 2 parts, a scalar s, and a vector v and is typically 
// written: q = s <vx, vy, vz>.  
//
// A unit-quaternion is one for which s^2+vx^2+vy^2+vz^2 = 1.  It can be 
// considered as a rotation by an angle theta about a unit-vector V in space where 
//
//         q = cos (theta/2) < v sin(theta/2)> 
//
// Q = Quaternion(X) is a unit-quaternion equivalent to X which can be any
// of:
//   - orthonormal rotation matrix.
//   - homogeneous transformation matrix (rotation part only).
//   - rotation angle and vector
// 
// To construct a quaternion from various other orientation representations.
//
// Q = Quaternion() is the identitity quaternion 1<0,0,0> representing a null rotation.
//
// Q = Quaternion(Q1) is a copy of the quaternion Q1
//
// Q = Quaternion([S V1 V2 V3]) is a quaternion formed by specifying directly its 4 elements
//
// Q = Quaternion(S) is a quaternion formed from the scalar S and zero vector part: S<0,0,0>
//
// Q = Quaternion(V) is a pure quaternion with the specified vector part: 0<V>
//
// Q = Quaternion(TH, V) is a unit-quaternion corresponding to rotation of TH about the 
// vector V.
//
// Q = Quaternion(R) is a unit-quaternion corresponding to the orthonormal rotation 
// matrix R.  If R (3x3xN) is a sequence then Q (Nx1) is a vector of Quaternions 
// corresponding to the elements of R.
//
// Q = Quaternion(T) is a unit-quaternion equivalent to the rotational
// part of the homogeneous transform T. If T (4x4xN) is a sequence then Q (Nx1) is a 
// vector of Quaternions corresponding to the elements of T.


function q = Quaternion(a1,a2)
    nargin = argn(2);
    //pause;
    if nargin==0 then
        q.v = [0,0,0];
        q.s = 1;
    elseif nargin==1 then
        if isvec(a1, 4) then 
            // q=Quaternion([s v1 v2 v3]) from 4 elements
            q.s = a1(1);
            q.v = a1(2:4);
        elseif isquaternion(a1)
            q = a1;
        elseif isrot(a1) | ishomog(a1)
            // q=Quaternion(R)  from a 3x3 or 4x4 matrix
            if ndims(a1)>2  // 3D matrix
                for i=1:size(a1,3)
                    qt = tr2q(a1(:,:,i));
                    q(i) = qt;
                end
            else
                q = tr2q(a1);
            end
        elseif isequal(size(a1),[1 3])
            //   Q = Quaternion(v)       from a 1 x 3 vector
             q.s = 0;
             q.v = a1;
        elseif isequal(size(a1),[3 1])
            //   Q = Quaternion(v)       from a 3 x 1 vector
             q.s = 0;
             q.v = a1';             
        elseif length(a1) == 1
            //   Q = Quaternion(s)       from a scalar
                    q.s = a1(1);
                    q.v = [0 0 0];
        else
            error('unknown dimension of input');
         end            
    elseif nargin == 2
         if isscalar(a1) & isvector(a2)
            //   Q = Quaternion(theta, v)    from vector plus angle
            q.s = cos(a1/2);
            q.v = sin(a1/2)*unit(a2(:)');
        else
            error ('bad argument to quaternion constructor');
        end
            
    end      // if nargin == 0          

endfunction

function q = quaternion(a1,a2)
    nargin = argn(2);
    if nargin==0 then
        q = Quaternion();
    elseif nargin == 1 then
        q = Quaternion(a1);
    elseif nargin == 2 then
        q = Quaternion(a1,a2);
    end

endfunction



