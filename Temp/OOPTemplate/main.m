clear; clc;
n = 6;
m = 2;
dt  = 1e-2;
t = 0:dt:10;
x = randn([6, 1]);
K0 = zeros(2, 6);
diagQ = diag([1, 1, 0.1, 0.1, 0.1, 0.1]);
aoc = AdaptiveOptimalControl(n, m, dt, x, K0, diagQ);
uF = zeros(2, 1);
iFinal = length(t);
for i = 1:300
    x = UpdateDynamic_ver2(x, uF, dt);
    aoc.CollectData(x, uF);
    aoc.UpdateControlLaw();
    uF = aoc.OptimalControlLaw(x, t(i));
    % waitbar(i/iFinal);
end
