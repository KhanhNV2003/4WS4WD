clear; clc;
l = 0.3;
d = 0.2;
x0 = 0;
y0 = 10;
psi0 = 0;

Robot = Model4WS4WD(l, d, x0, y0, psi0);
Robot.UpdatePosition(1, 0, 1, 0, 1, 0, 1, 0);

