// hw31_verify.sce   verify homework 3 prob 1 
// www.controlsystemslab.com   July 2012
// Note: read variable q from workspace
// q = [theta1 d2 theta3]

a3 = 1;
s1 = sin(q(1));
c1 = cos(q(1));
s3 = sin(q(3));
c3 = cos(q(3));

s13 = sin(q(1)+q(3));
c13 = cos(q(1)+q(3));

d2 = q(2);

T30_a = clean([-c1*c3+s1*s3, c1*s3+s1*c3, 0, -a3*c1*c3+a3*s1*s3+d2*s1;
         -s1*c3-c1*s3, s1*s3-c1*c3, 0, -a3*s1*c3-a3*c1*s3-d2*c1;
         0, 0, 1, 0;
         0, 0, 0, 1]);
 T30_b = clean([-c13, s13, 0, -a3*c13+d2*s1;
          -s13, -c13, 0, a3*s13-d2*c1;
          0, 0, 1, 0;
          0, 0, 0, 1]);
          