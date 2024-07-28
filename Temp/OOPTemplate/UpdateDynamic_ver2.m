function xNext = UpdateDynamic_ver2(xPre, u, dt)
A = [-0.5125, -0.0248, 0.0741, 0.0089, 0, 0;
     101.5873, -7.2651, 2.7608, 2.8068, 0, 0;
     0.0704, 0.0085, -0.0741, -0.0089, 0, 0.02;
     0.0878, 0.2672, 0, -0.3674, 0.0044, 0.3962;
     -1.8414, 0.0990, 0, 0, -0.0343, -0.0330;
     0, 0, 0, -359, 187.5364, -87.0316];
B = [-0.0042, -1.0360, 0.0042, 0.1261, 0, 0;
     0.0064, 1.5849, 0, 0, -0.0168, 0]';

dx = @(x,u)( A*x + B*u); % dx/dt = f(x,u)

% Runge-Kutta 4 integration
k1 = dx(xPre,         u);
k2 = dx(xPre + dt./2*k1, u);
k3 = dx(xPre + dt./2*k2, u);
k4 = dx(xPre + dt*k3,   u);
xNext = xPre + dt./6*(k1 + 2*k2 + 2*k3 + k4); 

end