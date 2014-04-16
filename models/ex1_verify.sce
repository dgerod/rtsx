// ex1_verify.sce  Verify answer of Example 1 that is solved by hand
// www.controlsystemslab.com   July 2012
// Note: read variable q from workspace
// q = [theta1 d2 theta3]

s1 = sin(q(1));c1 = cos(q(1));s3 = sin(q(3));c3 = cos(q(3));
d2 = q(2);
Ta = [s1*s3,   s1*c3,   c1,   s1*s3-s1*d2; ..
      -c1*s3,  -c1*c3,   s1,  -c1*s3+c1*d2; ..
      c3,        -s3,     0,      c3+1; ..
      0,            0,     0,        1 ]
 
