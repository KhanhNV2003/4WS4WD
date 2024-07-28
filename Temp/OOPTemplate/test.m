clear; clc;
l = 33;
K = zeros(2, 6);
Q = diag([1, 1, 0.1, 0.1, 0.1, 0.1]);
vecQ = [1; zeros(5, 1);
        0; 1; zeros(4, 1);
        zeros(2, 1); 0.1; zeros(3, 1);
        zeros(3, 1); 0.1; zeros(2, 1);
        zeros(4, 1); 0.1; 0;
        zeros(5, 1); 0.1];
R = eye(2, 2);
I = eye(6, 6);
dt  = 1e-2;
t = 0:dt:2;
omega = rand([1, 100]);
iFinal = length(t);
x = randn([6, 1]);
xBar = zeros(21, l+1);
xPre = [zeros(6, l), x];
uPre = zeros(2, l+1);
omega1 = randi([-500, 500], 1, 100);
omega2 = randi([-500, 500], 1, 100);
% 
for i=1:6
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
    disp("Rank Theta:");
    disp(rank(Theta));
    disp("Rank IxxIxu:");
    disp(rank([Ixx, Ixu]));
    % if (rank([Ixx, Ixu]) == 33)
    %     break;
    % end
end
Xi = -Ixx*vecQ;
rank([Ixx, Ixu])
% Ans = (Theta'*Theta)^-1*Theta'*Xi;
rank(Theta)
% pinv(Theta)
% 