// p560_gravplot.sce  plot 3-D surface of gravity for Puma 560
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
        g = gravload(p560,[0 Q2(i,j) Q3(i,j) 0 0 0]);
         g2(i,j) = g(2);
        g3(i,j) = g(3);
        printf('\r%d',count);
        count = count+1;
    end
 end
// figure; surf(Q2, Q3, g2); 
// figure; surf(Q2, Q3, g3);

subplot(121), surf(Q2, Q3, g2); 
xlabel('Q2'); ylabel('Q3'); zlabel('g2');
subplot(122), surf(Q2, Q3, g3);
xlabel('Q2'); ylabel('Q3'); zlabel('g3');
