ece-499-590-midterm2
====================
FK.m contains a generic Forward Kinematics solver.  Pass in a vector of Lengths and a vector of angles and it will return an [x;y;z] coordinate.  X = FK(l,th)

FK_Set1_Set2.m contains the set1 and set2 information and will call FK() to solve for set1 and set2

IK_Jacobian.m contains an Inverse Kinematic solver using the Jacobian Method.  Pass in a vector of lengths, angles, angle step sizes, desired end effector, and a scalar of acceptable error, and it will return the angles to reach desired end effector.  THETA_final = FK(l,th,d_th,ee,err).  Note that the end effector is expected to be in three dimensions.
Read Jacobian_IK_Implementation.txt for more details and the outputs to the given sets.
