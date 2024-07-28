clear; clc;
l = 33;
n = 6;
m = 2;
K = zeros(2, 6);
diagQ = [1, 1, 0.1, 0.1, 0.1, 0.1];
vecQ = [];
for i=1:6
    vecQ = [vecQ; zeros(i-1, 1); diagQ(i); zeros(6-i, 1)];
end
R = eye(m, m);
I = eye(n, n);
dt  = 1e-2;
t = 0:dt:2;
x = randn([6, 1]);
xBar = zeros(21, l+1);
xPre = [zeros(6, l), x];
uPre = zeros(2, l+1);
omega1 = randi([-500, 500], 1, 100);
omega2 = randi([-500, 500], 1, 100);
% 
for i=1:33
    xBarCur = [];
    for j = 1:6
        for k = j:6
            xBarCur = [xBarCur; x(j)*x(k)];
        end
    end
    xBar = [xBar(:, 2:end), xBarCur];
    deltaXX = zeros(l, 21);
    for j = 1:l
        deltaXX(j, :) = [xBar(:, j+1) - xBar(:, j)];
    end
    Ixx = zeros(l, 36);
    for j = 1:l
        Ixx(j, :) = [(kron(xPre(:, j+1), xPre(:, j+1)) + kron(xPre(:, j), xPre(:, j)))*dt/2];
    end
    Ixu = zeros(l, 12);
    for j = 1:l
        Ixu(j, :) = [(kron(xPre(:, j+1), uPre(:, j+1)) + kron(xPre(:, j), uPre(:, j)))*dt/2];
    end
    Theta = [deltaXX, -2*Ixx*(kron(eye(6), K'*R)) - 2*Ixu*(kron(eye(6), R))];
    
    e1 = 100*sum(sin(omega1.*t(i)));
    e2 = 100*sum(sin(omega2.*t(i)));
    u = K*x;
    uF = u + [e1; e2];
    x = UpdateDynamic(x, uF, dt);
    xPre = [xPre(:, 2:end), x];
    uPre = [uPre(:, 2:end), uF];
    % if (rank([Ixx, Ixu]) == 33)
    %     break;
    % end
end
Xi = -Ixx*vecQ;
% rank([Ixx, Ixu])
% Ans = (Theta'*Theta)^-1*Theta'*Xi;
% rank(Theta)
Sol = pinv(Theta)*Xi;
% 
k = 0;
PCur = zeros(6, 6);                
for i = 1:6
    for j = i:6
        k = k + 1;
        PCur(i, j) = Sol(k);
        PCur(j, i) = Sol(k);
    end
end

for i = 1:2
    for j = 1:6
        k = k + 1;
        K(i, j)= Sol(k);
    end
end
%%
clc
n = 6;
m = 2;
dt  = 1e-2;
t = 0:dt:100;
x = randn([6, 1]);
K0 = zeros(2, 6);
diagQ = [1, 1, 0.1, 0.1, 0.1, 0.1];
aoc = AdaptiveOptimalControl(n, m, dt, x, K0, diagQ);
uF = zeros(2, 1);
for i = 1:length(t)
    x = UpdateDynamic(x, uF);
    aoc.CollectData(x, uF);
    aoc.UpdateControlLaw();
    uF = aoc.OptimalControlLaw(x, t(i));
end
%%
k = 0;
                for i = 1:6
                    for j = i:6
                        k = k + 1
                    end
                end


%%
A = [-0.4125, -0.0248, 0.0741, 0.0089, 0, 0;
     101.5873, -7.2651, 2.7608, 2.8068, 0, 0;
     0.0704, 0.0085, -0.0741, -0.0089, 0, 0.02;
     0.0878, 0.2672, 0, -0.3674, 0.0044, 0.3962;
     -1.8414, 0.0990, 0, 0, -0.0343, -0.0330;
     0, 0, 0, -359, 187.5364, -87.0316];
B = [-0.0042, -1.0360, 0.0042, 0.1261, 0, 0;
     0.0064, 1.5849, 0, 0, -0.0168, 0]';
Q = diag([1, 1, 0.1, 0.1, 0.1, 0.1]);
R = eye(2);

lqr(A, B, Q, R)

%%
omega = 18.896807023199038;
zeta = 0.004221651082263;
A = [0, 0, 1, 0, zeros(1, 4);
     0, 0, 0, 1, zeros(1, 4);
     -omega^2, 0, -2*omega*zeta, 0, zeros(1, 4);
     0, -omega^2, 0, -2*omega*zeta, zeros(1, 4);
     zeros(1, 4), 0, 0, 1, 0;
     zeros(1, 4), 0, 0, 0, 1;
     zeros(1, 8);
     zeros(1, 8)];
B = [0, 0, 1, 0, 0, 0, -1, 0;
     0, 0, 0, 1, 0, 0, 0, -1]';
Q = diag([1, 1, 0.1, 0.1, 10, 10, .1, .1]);
R = eye(2);

lqr(A, B, Q, R)
%%
Q = diag([1,  2, 3, 4])
Q(:)





