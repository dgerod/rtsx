// depth_estimator.sci  
// used by IBVS4.sci

function [Zest] = depth_estimator(cam, vs, uv)
            //IBVS.depth_estimator Estimate point depth
            //
            // [Zest,Zreal] = depth_estimator(UV) are the estimated and true world 
            // point depth based on current feature coordinates UV (2xN).
            
            if isempty(vs.p_old)
                Zest = [];

                return;
            end

            // compute Jacobian for unit depth, z=1
            J = visjac_p(cam, uv, 1);
            Jv = J(:,1:3);  // velocity part, depends on 1/z
            Jw = J(:,4:6);  // rotational part, indepedent of 1/z
            
            // estimate image plane velocity
            uv_d =  uv(:) - vs.p_old(:);
            
            // estimate coefficients for A (1/z) = B
            B = uv_d - Jw*vs.v_old(4:6);
            A = Jv * vs.v_old(1:3);

            AA = zeros(numcols(uv), numcols(uv)/2);
            for ix=1:numcols(uv)
                AA(ix*2-1:ix*2,ix) = A(ix*2-1:ix*2);
            end
            eta = AA\B;          // least squares solution
            
            eta2 = A(1:2) \ B(1:2);
            //pause;
            // first order smoothing
            vstheta = vs.theta;
            vssmoothing = vs.smoothing;
            vstheta = (1-vssmoothing) * 1 ./eta' + vssmoothing * vstheta;
            Zest = vstheta;

            // true depth
            //P_CT = homtrans(inv(vs.Tcam), vs.P);
            //Ztrue = P_CT(3,:);

//            if vs.verbose
//                fprintf('depth //.4g, est depth //.4g, rls depth //.4g\n', ...
//                    Ztrue, 1/eta, Zest);
//            end
        
endfunction
