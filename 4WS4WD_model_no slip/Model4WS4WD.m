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

        end
        
        function oj = UpdatePosition(oj, v1, phi1, v2, phi2, v3, phi3, v4, phi4)
            % This function update 
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
                x = eta(1); y = eta(2); psi = eta(3);
                R = [cos(psi), -sin(psi), 0;
                        sin(psi), cos(psi), 0;
                        0, 0, 1]; 
                detadt = (R*pinv(oj.W)*V);
            end
            oj.x = etaNext(1); oj.y = etaNext(2); oj.psi = etaNext(3);
        end

        
    end
end

