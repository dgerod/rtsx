//UNIT Unitize a vector
//
// VN = UNIT(V) is a unit vector parallel to V.
//
// Note::
// - Reports error for the case where norm(V) is zero.


function u = unit(v)
    n = norm(v, 'fro');
    if n < %eps
        error('RTB:unit:zero_norm', 'vector has zero norm');
    end

	u = v / n;
endfunction