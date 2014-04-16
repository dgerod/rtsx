// p560_inertiaplot.sce  plot 3-D surface of gravity for Puma 560
stepsize = 0.1;
[Q2,Q3] = meshgrid(-pi:stepsize:pi, -pi:stepsize:pi);
nloops = numcols(Q2)*numcols(Q3);
printf('Generating data in %d iterations\n', nloops);
printf('\n');
count = 0;
g2 = _zeros(size(Q2));
g3 = _zeros(size(Q3));
 for i=1:numcols(Q2),
     for j=1:numcols(Q3),
        D = inertia(p560,[0 Q2(i,j) Q3(i,j) 0 0 0]);
        D11(i,j) = D(1,1);
        D12(i,j) = D(1,2);
        printf('\r%d',count);
        count = count+1;
    end
 end
 subplot(121), surf(Q2, Q3, D11); 
 xlabel('Q2'); ylabel('Q3'); zlabel('D11');
 subplot(122), surf(Q2, Q3, D12);
xlabel('Q2'); ylabel('Q3'); zlabel('D12');
