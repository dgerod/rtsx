// ============================================================================
// check_kinematics_algorithms.sce
// ============================================================================

// Kinematics parameters (rf,lf,le,re)
exec(RTSX_ROOT + 'models/delta2_cugd2_mdl.sce',-1);
// Position to solve
p1 = [50,0,-700]';

// ----------------------------------------------------------------------------

// Inverse kinematics ---

mprintf( "\np1 = %f,%f,%f\n",p1(1),p1(2),p1(3) );
mprintf( "tcp0 --> [IK] --> joints\n");
tcp0 = p1;
[ret,joints] = d2rInvKinem(kp, tcp0)
mprintf( "joints = %f,%f\n\n",joints(1),joints(2));

// Direct kinematics ---

mprintf( "joints = %f,%f\n",joints(1),joints(2));
mprintf( "joints --> [DK] --> tcp0\n");
[ret,p2] = d2rDirKinem(kp, joints)
mprintf( "p2 = %f,%f,%f\n",p2(1),p2(2),p2(3) );

// ============================================================================
