function qp = qpower(q, p)
//Quaternion.mpower Raise quaternion to integer power
//
// Q^N is the quaternion Q raised to the integer power N.
//
// Notes::
// - Computed by repeated multiplication.
//

    // check that exponent is an integer
    if (p - floor(p)) ~= 0
        error('quaternion exponent must be integer');
    end

    qp = q;

    // multiply by itself so many times
    for i = 2:abs(p)
        qp = qmult(qp,q);
    end

    // if exponent was negative, invert it
    if p<0
        qp = qinv(qp);
    end
endfunction



