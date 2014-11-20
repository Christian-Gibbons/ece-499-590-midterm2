%this file relies on FK.m for the Forward Kinematic solver.  This just inputs the given sets into it.
l1 = [0.1 0.1 0.1 0.1 0.1 0.1 0.1 0.1 0.1];
th1 = l1;
set1 = FK(l1,th1)
l2 = [0.3 0.2 0.1 0.1 0.2 0.1 0.3 0.1 0.1];
th2 = [0.4 0.6 1.2 0.5 0.1 0.7 0.3 0.2 0.1];
set2 = FK(l2,th2)
