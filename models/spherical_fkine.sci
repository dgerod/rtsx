// sph_fkine.sci  forward kinemetic of spherical robot

function T=sph_fkine(q,d1)
    nargin = argn(2);
    if nargin==1 then
        d1 = 1;
    end
    c1 = cos(q(1));
    s1 = sin(q(1));
    c2 = cos(q(2));
    s2 = sin(q(2));
    d3 = q(3);
    T = clean([c1*c2, s1, c1*s2, c1*s2*d3; s1*c2, -c1, s1*s2, s1*s2*d3; s2, 0, -c2, -c2*d3+d1;0 0 0 1]);
endfunction
