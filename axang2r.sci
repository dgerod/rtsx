//AXANG2R Convert angle and vector orientation to a rotation matrix
//
// R = AXANG2R(K,THETA, 'options') is an rthonormal rotation matrix, R, 
// equivalent to a rotation of THETA about the vector K.
//
// See also eul2r, rpy2r.


function R = axang2r(k,theta, varargin)
    opt.deg = 0;

    // process varargin 
    numopts=length(varargin); // find number of options
    if numopts>0 then
        for i=1:numopts
            if varargin(i)== 'deg' then opt.deg = 1;
            end
        end
    end
    // optionally convert from degrees
    if opt.deg then
        d2r = %pi/180.0;
        theta = d2r*theta;
    end

    if norm(k)~=1  then k=k/norm(k);   // normalize 
    end
	kx = k(1); ky = k(2); kz = k(3);

    kxy = sqrt(kx^2 + ky^2);
    Rza = [kx/kxy, -ky/kxy, 0; ky/kxy, kx/kxy, 0;0, 0, 1];
    Ryb = [kz, 0, kxy; 0, 1, 0;-kxy, 0, kz];
    R = clean(Rza*Ryb*rotz(theta)*Ryb'*Rza');

 
endfunction
