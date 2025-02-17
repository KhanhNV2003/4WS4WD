classdef Model4WS4WD <handle
    %UNTITLED Summary of this class goes here
    %   Detailed explanation goes here
    
    properties
        % XY - global coordinate
        % XBYB - local coordinate attach with the robot
        x % position of the robot along X-axis
        y % position of the robot along Y-axis
        psi % angle of the robot in the XY-axes
        l % body length (distance from the centre of the robot to the wheel along X-axis)
        d % body width (distance from the centre of the robot to the wheel along Y-axis)
        W % W robot coefficient matrix 
        dt % sampling time
        v1x % mearsurable variable
        v1y % measurable variable
        v2y % measurable variable
    end
    
    methods
        function oj = Model4WS4WD(l, d, x0, y0, psi0)
            %Construct an instance of this class 4WS4WD
            %   Initialize a 4WS4WD robot
            oj.dt = 1e-3;
            oj.l = l; % assign body length
            oj.d = d; % assign body width
            oj.x = x0; % assign initial coordinate on the X-axis
            oj.y = y0; % assign initial coordinate on the Y-axis
            oj.psi = psi0; % assign initial angle on the XY-axes
            oj.W = [1, 0, -l;
                    0, 1, d;
                    1, 0, -l;
                    0, 1, -d;
                    1, 0, l;
                    0, 1, -d;
                    1, 0, l;
                    0, 1, d]; % assign matrix W
            oj.v1x = 0;
            oj.v1y = 0;
            oj.v2y = 0;

        end
        
        function oj = UpdatePosition(oj, input)
            % This function update 
            v1 = input(1);
            phi1 = input(2);
            v2 = input(3);
            phi2 = input(4);
            v3 = input(5);
            phi3 = input(6);
            v4 = input(7);
            phi4 = input(8);
            %   Detailed explanation goes here
            v1x = v1*cos(phi1); % velocity of wheel 1 along XBYB-axis
            v1y = v1*sin(phi1); % velocity of wheel 1 along YB-axis
            v2x = v2*cos(phi2); % velocity of wheel 2 along XBYB-axis
            v2y = v2*sin(phi2); % velocity of wheel 2 along YB-axis
            v3x = v3*cos(phi3); % velocity of wheel 3 along XBYB-axis
            v3y = v3*sin(phi3); % velocity of wheel 3 along YB-axis
            v4x = v4*cos(phi4); % velocity of wheel 4 along XBYB-axis
            v4y = v4*sin(phi4); % velocity of wheel 4 along YB-axis
            V = [v1x; v1y; v2x; v2y; v3x; v3y; v4x; v4y]; 
            eta = [oj.x; oj.y; oj.psi]; 
            
            % Runge-Kutta 4 integration
            k1 = odefcn(eta, V);
            k2 = odefcn(eta + oj.dt/2*k1, V);
            k3 = odefcn(eta + oj.dt/2*k2, V);
            k4 = odefcn(eta + oj.dt*k3,   V);
            etaNext = eta + oj.dt/6*(k1 + 2*k2 + 2*k3 + k4); % Calculate next position
            
            function detadt = odefcn(eta, V)
                xCur = eta(1); yCur = eta(2); psiCur = eta(3);
                R = [cos(psiCur), -sin(psiCur), 0;
                        sin(psiCur), cos(psiCur), 0;
                        0, 0, 1]; 
                detadt = (R*pinv(oj.W)*V);
            end
            oj.x = etaNext(1); oj.y = etaNext(2); oj.psi = etaNext(3);
            oj.v1x = v1x;
            oj.v1y = v1y;
            oj.v2y = v2y;
        end

        function out = Controller(oj)

            vc = 2;
            b = 10;

            % Control parameter
            ku = 1;
            beta = 2;

            % Calculate desired steering angle
            psid = -atan2(oj.y, b);
            dpsid = b^2/(b*(b^2 + oj.y^2))*vc*sin(psid);

            %  Calculate system variables
            phi1 = atan2(oj.v1y, oj.v1x);
            dpsi = vc/(oj.d*sin(phi1) + oj.l*cos(phi1));
            v2 = sqrt(oj.v1x^2 + oj.v2y^2);
            phi2 = atan2(sqrt(v2^2 - oj.v1x^2), oj.v1y);
            phi3 = atan2(oj.v2y, (oj.v1x + (oj.v1y - oj.v2y)*oj.l/oj.d));
            phi4 = atan2(oj.v1y, (oj.v1x + (oj.v1y - oj.v2y)*oj.l/oj.d));

            % Calculate control law for wheel 1
            phi1d = .5*atan(beta*(psid - oj.psi));
            dphi1d = .5/(1 + (beta*(psid - oj.psi))^2 )*beta*(dpsid - dpsi);
            u1 = ku*(phi1d - phi1) + dphi1d;

            % Calculate control law for wheel 2, 3, 4
            phi2d = phi1d;
            phi3d = phi1d;
            phi4d = phi1d;

            dphi2d = dphi1d;
            dphi3d = dphi1d;
            dphi4d = dphi1d;

            u2 = ku*(phi2d - phi2) + dphi2d;
            u3 = ku*(phi3d - phi3) + dphi3d;
            u4 = ku*(phi4d - phi4) + dphi4d;

            % Constraint for maximum angle
            u1 = min(max(u1, -pi/4), pi/4);
            u2 = min(max(u2, -pi/4), pi/4);
            u3 = min(max(u3, -pi/4), pi/4);
            u4 = min(max(u4, -pi/4), pi/4);

            out = [vc, u1, vc, u2, vc, u3, vc, u4];
        end

        
    end
end

