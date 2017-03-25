// ============================================================================
// Kinematics algorithms of delta-2 robot
// ============================================================================

// Load robot model 
exec(RTSX_ROOT + 'models/delta2_cugd2_mdl.sce',-1);

// Inverse kinematics
tcp0 = [50, 0, -700]';
expected_joints = [0.0337310, 0.2247364]';
granularity = 1e-5;

[ret, joints] = d2rInvKinem(delta2_robot, tcp0)
assert_checkalmostequal(joints, expected_joints, granularity);

// ============================================================================
