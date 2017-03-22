// ============================================================================
// check_kinematics_algorithms.sce
// ============================================================================

// Kinematics parameters (h,l1,l2)
L = [30,30,40];
// Position to solve
p = [30,0,10]';

// -----------------------------------------------------------------------------

// Inverse kinematics of Scara robot ---

[ok,J] = scrInvKinem(L, p);
j1 = J(:,1);
j2 = J(:,2);

// Direct kinematics of Scara robot ---

[ok_1, tcp0_1] = scrDirKinem(L, j1);
[ok_2, tcp0_2] = scrDirKinem(L, j2);

// Print solution ---

disp(L,"Kinem params");
disp(p,"Target position");

solution = [j1, tcp0_1, j2, tcp0_2];
disp(solution,"Solution");

// =============================================================================
