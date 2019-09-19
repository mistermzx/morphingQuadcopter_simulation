classdef Gap
    properties
        s_GE; % Position of center of gap in earth frame
        z_G; % direction of gap in world frame
        l; % length of gap
        h; % height of gap
        w; % width of gap
        T_GE; % Transformation matrix
    end
    methods 
        function obj = Gap(gapCenter, gapDirection, gapLength, gapHeight, gapWidth)
            obj.s_GE = gapCenter;
            obj.z_G = gapDirection;
            obj.l = gapLength;
            obj.h = gapHeight;
            obj.w = gapWidth;
            % Calculate orientation
            psi = atan2(obj.z_G(2), obj.z_G(1));
            T_CE = Rotation.fromAxisAngle([0; 0; 1], psi);
            T_CE = T_CE.to_trans_matrix();
            theta = atan2(dot(T_CE*obj.z_G, [1; 0; 0]), dot(T_CE*obj.z_G, [0; 0; 1]));
            T_GC = Rotation.fromAxisAngle([0; 1; 0], theta);
            T_GC = T_GC.to_trans_matrix();
            obj.T_GE = T_GC*T_CE;
        end
       
        function [s_BE_0, Ds_BE_0, T_f] = generate_trajectory(obj)
            % This function generates the minimum initial velocity trajectory through a
            % rectangular prism of size l x h x w, where l is the length, h height, and
            % w is the width of the quadcopter when folded (assume known).
            %   Inputs:  s_GE = position of center of gap in world frame
            %            z_G = direction of gap in world frame
            %            l = length of gap
            %            h = height of gap
            %   Outputs: s_BE = initial position of quadcopter in world frame
            %            Ds_BE = initial velocity of quadcopter in world frame

            s_GE= obj.s_GE;
            z_G = obj.z_G;
            l = obj.l;
            h = obj.h;
            % Given
            T_fold = 0.2; % Time to fold arms [s]

            % Formulate transition matrices from world frame to gap frame

            % Let farme C be an intermediate frame that alignes x_C with the x_G-z_G
            % plane
            psi = atan2(z_G(2), z_G(1));
            T_CE = Rotation.fromAxisAngle([0; 0; 1], psi);
            T_CE = T_CE.to_trans_matrix();
            theta = atan2(dot(T_CE*z_G, [1; 0; 0]), dot(T_CE*z_G, [0; 0; 1]));
            T_GC = Rotation.fromAxisAngle([0; 1; 0], theta);
            T_GC = T_GC.to_trans_matrix();
            T_GE = T_GC*T_CE;
            T_EG = T_GE';

            % Formulate objective and constraints
            % x = [s_BE_0, Ds_BE_0, T_f]

            % Penalize large initial velocity
            obj = @(x) norm(x(4:6));

            % Formulate constraints on passing through gap
            nonlcon = @(x) constraint_fn(x, l, h, T_GE, T_fold);
            % We must start on the correct side of the gap (s_BE_0(3) < -l)
            A = [0 0 1 0 0 0 0];
            b = -l;
            % We must pass through the front face of the gap at T_fold
            % s_BG_0(1) + Ds_BG_0(1)*T_fold + T_GE*g*T_fold^2/2 on [-h/2, h/2]
            g_G = T_GE*[0; 0; -9.81]; % gravity in G frame
            A = [A; -1, 0, 0, -T_fold, 0, 0, 0;
                    1, 0, 0, T_fold, 0, 0, 0];
            b = [b; h/2 + g_G(1)*(T_fold^2)/2;
                     h/2 - g_G(1)*(T_fold^2)/2];
            % We must leave the gap after we enter (T_f > T_fold)
            A = [A; 0 0 0 0 0 0 -1];
            b = [b; T_fold];
            % s_BG_0(2) and Ds_BG_0(2) must equal zero (i.e. fly through center of gap)
            Aeq = [0 1 0 0 0 0 0;
                   0 0 0 0 1 0 0];
            beq = zeros(2,1);
            % We must pass through the front face of the gap at T_fold
            % s_BG_0(3) + Ds_BG_0(3)*T_fold + T_GE*g*T_fold^2/2 == -l/2
            Aeq = [Aeq; 0, 0, 1, 0, 0, T_fold, 0];
            beq = [beq; -l/2 - g_G(3)*(T_fold^2)/2];

            % Initial guess
            x0 = [0, 0, -2*l, 0, 0, 1, T_fold+0.1];

            % Solve optimization
            opt = optimoptions('fmincon','Display','iter');
            x = fmincon(obj, x0, A, b, Aeq, beq, [], [], nonlcon, opt);
            s_BG_0 = x(1:3)';
            Ds_BG_0 = x(4:6)';
            T_f = x(7);

            % Convert s_BG_0 and Ds_BG_0 into s_BE_0 and Ds_BE_0 in the world frame
            s_BE_0 = T_EG*s_BG_0 + s_GE;
            Ds_BE_0 = T_EG*Ds_BG_0;

        end
       
    end
end