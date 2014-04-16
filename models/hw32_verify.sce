// hw32_verify.sce   verify homework 3 prob 2
// www.controlsystemslab.com   July 2012
// Note: read variable q from workspace
// q = [theta1 theata2 theta3]

d1 = 1;
a2 = 1;
a3 = 1;
s1 = sin(q(1));
c1 = cos(q(1));
s2 = sin(q(2));
c2 = cos(q(2));
s3 = sin(q(3));
c3 = cos(q(3));

s23 = sin(q(2)+q(3));
c23 = cos(q(2)+q(3));



T30_a = clean([c1*c2*c3-c1*s2*s3, -c1*c2*s3-c1*s2*c3, s1, a3*c1*c2*c3-a3*c1*s2*s3+a2*c1*c2;
         s1*c2*c3-s1*s2*s3, -s1*c2*s3-s1*s2*c3, -c1, a3*s1*c2*c3-a3*s1*s2*s3+a2*s1*c2;
         s2*c3+c2*s3, -s2*c3+c2*c3, 0, a3*s2*c3+a3*c2*s3+a2*s2+d1;
         0, 0, 0, 1]);
T30_b = clean([c1*c23, -c1*s23, s1, a3*c1*c23+a2*c1*c2;
          s1*c23, -s1*s23, -c1, a3*s1*c23+a2*s1*c2;
          s23, c23, 0, a3*s23+a2*s2+d1;
          0, 0, 0, 1]);
          