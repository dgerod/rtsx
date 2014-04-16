// sph_fkine.sci  forward kinemetic of spherical robot

function T=sphwrist_fkine(q,d1,d3)
    nargin = argn(2);
    if nargin==1 then
        d1 = 1;
        d3 = 1;
    elseif nargin==2 then
        d3 = 1;
    end
    c1 = cos(q(1));
    s1 = sin(q(1));
    c2 = cos(q(2));
    s2 = sin(q(2));
    c3 = cos(q(3));
    s3 = sin(q(3));
    T = clean([c1*c2*c3-s1*s3, -c1*c2*s3-s1*c3, -c1*s2, -c1*s2*d3; 
    s1*c2*s3+c1*s3, -s1*c2*s3+c1*c3, -s1*s2, -s1*s2*d3;
     s2*c3, -s2*s3, c2, c2*d3+d1;
     0 0 0 1]);
endfunction
