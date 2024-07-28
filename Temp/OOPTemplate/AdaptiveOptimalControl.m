classdef AdaptiveOptimalControl <handle
    %UNTITLED Summary of this class goes here
    %   Detailed explanation goes here
    
    properties
        n                                       %len of state variable x
        m                                       %len of control input u
        l                                       %minimum data need to collect
        vecQk                                    %vecto of diagonal matrix Qk
        K                                       %optimal control law K   
        P                                       %solution matrix P
        Q                                       %weighted matrix Q
        dt                                      %sample time
        R                                       %matrix R m*m
        I                                       %eye matrix I n*n
        xBar
        xPre
        uPre
        omega1 = randi([-500, 500], 1, 100);
        omega2 = randi([-500, 500], 1, 100);
        deltaXX;
        Ixx;
        Ixu;
        Qk;
        Theta
        Xi
        rankIxxIxu = 0;
        rankTheta = 0;
        endIteration = 0;
    end
    
    methods
        function oj = AdaptiveOptimalControl(n, m, dt, xInit, K0, Q)
            %UNTITLED Construct an instance of this class
            %   Detailed explanation goes here
            oj.n = n;
            oj.m = m;
            l = n*(n + 1)/2 + m*n;
            oj.l = l;
            % vecQ = [];
            % for i=1:6
            %     vecQ = [vecQ; zeros(i-1, 1); diagQ(i); zeros(6-i, 1)];
            % end
            % oj.vecQ = vecQ;
            oj.Q = Q;
            oj.K = K0;
            oj.dt = dt;
            oj.R = eye(m, m);
            oj.I = eye(n, n);
            oj.Qk = oj.Q + oj.K'*oj.R*oj.K;
            oj.vecQk = oj.Qk(:);
            oj.xBar = zeros(n*(n + 1)/2, l+1);
            oj.xPre = [zeros(n, l), xInit];
            oj.uPre = zeros(m, l+1);
            oj.deltaXX = zeros(l, n*(n + 1)/2);
            oj.Ixx = zeros(l, n*n);
            oj.Ixu = zeros(l, m*n);
            oj.P = zeros(n, n);
        end
        
        function oj = CollectData(oj, x, uF)
            %METHOD1 Summary of this method goes here
            %   Detailed explanation goes here
            %update xBar
            xBarCur = [];
            for i = 1:oj.n
                for j = i:oj.n
                    xBarCur = [xBarCur; x(i)*x(j)];
                end
            end
            oj.xBar = [oj.xBar(:, 2:end), xBarCur];
            
            %update deltaXX
            for i = 1:oj.l
                oj.deltaXX(i, :) = oj.xBar(:, i+1) - oj.xBar(:, i);
            end
            
            %update Ixx
            for i = 1:oj.l
                oj.Ixx(i, :) = (kron(oj.xPre(:, i+1), oj.xPre(:, i+1)) + kron(oj.xPre(:, i), oj.xPre(:, i)))*oj.dt/2;
            end

            %update Ixu
            for i = 1:oj.l
                oj.Ixu(i, :) = (kron(oj.xPre(:, i+1), oj.uPre(:, i+1)) + kron(oj.xPre(:, i), oj.uPre(:, i)))*oj.dt/2;
            end

            %update vecQk
            oj.Qk = oj.Q + oj.K'*oj.R*oj.K;
            oj.vecQk = oj.Qk(:);

            %update Theta
            oj.Theta = [oj.deltaXX, -2*oj.Ixx*(kron(oj.I, oj.K'*oj.R)) - 2*oj.Ixu*(kron(oj.I, oj.R))];

            %update Xi
            oj.Xi = -oj.Ixx*oj.vecQk;

            %update collected data
            oj.xPre = [oj.xPre(:, 2:end), x];
            oj.uPre = [oj.uPre(:, 2:end), uF];

            %update rank
            oj.rankIxxIxu = rank([oj.Ixx, oj.Ixu]);
            oj.rankTheta = rank(oj.Theta);
        end

        function oj = UpdateControlLaw(oj)
            if (oj.rankTheta == oj.l)&&(~oj.endIteration)
                Sol = pinv(oj.Theta)*oj.Xi;
                k = 0;
                
                %update matrix P
                PCur = zeros(oj.n, oj.n);                
                for i = 1:oj.n
                    for j = i:oj.n
                        k = k + 1;
                        PCur(i, j) = Sol(k);
                        PCur(j, i) = Sol(k);
                    end
                end
                if (norm(PCur - oj.P) < 0.03)
                    oj.endIteration = 1;
                    disp("End of Iteration");
                end
                oj.P = PCur;

                %update matrix K
                for j = 1:oj.n
                    for i = 1:oj.m
                        k = k + 1;
                        oj.K(i, j h       fgffdfdfdscx)= Sol(k);
                    end
                end

            end
        end

        function output = OptimalControlLaw(oj, x, t)
            if oj.endIteration
                output = -oj.K*x;
                disp('ye');
            else
                e1 = 10*sum(sin(oj.omega1.*t));
                e2 = 10*sum(sin(oj.omega2.*t));
                output = -oj.K*x + [e1; e2];
            end
        end
    end
end

