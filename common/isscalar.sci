// isscalar.sci  test if argument is scalar

function h=isscalar(x)
   h=type(x)== 1 & isequal(size(x),[1 1]);
endfunction

