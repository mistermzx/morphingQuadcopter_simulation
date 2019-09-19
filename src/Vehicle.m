classdef Vehicle
    properties
        inertia;
        mass;
        pos;
        vel;
        att; % T_BE
        omega;
        angAcc;
        transAcc;
        armList;
        armNumber; %number of Arms
        fullyStretched;
        
        % for controller:
        motorThrust_min; % Threshold for folding
        hoverThrust; % max for hover
        
    end
    methods (Static)
        function M = skewSymmetric (v)
            M = [0 -v(3) v(2); v(3) 0 -v(1); -v(2) v(1) 0];
        end
        function y = roundVector (x)
            y = x;
            for i=1:length(x)
                if abs(x(i))<1e-8
                    y(i) = 0;
                end
            end
        end
    end
    
    methods
        function obj = Vehicle(inertiaMatrix, mass)
            obj.inertia = inertiaMatrix;
            obj.mass = mass;
            obj.pos = [0,0,0]';
            obj.vel = [0,0,0]';
            obj.att = Rotation(); %implemented in Quaternions
            obj.omega = [0,0,0]';
            obj.angAcc = [0,0,0]';
            obj.transAcc = [0,0,0]';
            obj.armList = {};
            obj.armNumber = 0;
            obj.fullyStretched = false;
            obj.motorThrust_min = 0;
         
        end
        
        function obj = addArm(obj, arm)
            obj.armList{end+1} = arm;
            obj.armNumber = obj.armNumber+1;
        end
        
        function obj = update(obj, dt)
            %Update Body (through Euler Integration):
            vel = obj.vel;
            att = obj.att;
            omega = obj.omega;
            obj.pos = obj.pos + vel*dt;
            obj.vel = vel + obj.transAcc*dt;
            att_change = Rotation.from_rotation_vector(omega*dt);
            obj.att = att.compose(att_change);
            obj.omega = omega + obj.angAcc*dt;
            %Update Arm
            for i = 1:obj.armNumber
                obj.armList{i} = obj.armList{i}.updateArm(dt);
            end
        end
        
        function x = calcSolution(obj)
            %Quantities of the Body
            T_BE = obj.att.to_trans_matrix();
            W_BI = Vehicle.skewSymmetric(obj.omega);
            g = [0 0 -9.81]';
           
            % Calculate Constants (left side) for body:
            C1 = -W_BI*obj.inertia*obj.omega;
            C2 = obj.mass*T_BE*g;
            b = [C1;C2];
            
            A1 = zeros(3, (2+obj.armNumber*3)*3);
            A1 (:, 1:3) = obj.inertia;
            A2 = zeros(3, (2+obj.armNumber*3)*3);
            A2 (:, 4:6) = obj.mass*T_BE;
            A3 = zeros(obj.armNumber*6, (2+obj.armNumber*3)*3);
            
            for i= 1:obj.armNumber    
                %Define Arm quantities
                arm = obj.armList{i};
                T_BA = arm.orientation.to_trans_matrix();
                T_AB = transpose(T_BA);
                S_AH = Vehicle.skewSymmetric(arm.s_AH);
                S_HB = Vehicle.skewSymmetric(arm.s_HB);
                S_PH = Vehicle.skewSymmetric(arm.s_PH);
                S_PA = Vehicle.skewSymmetric(arm.s_PH-arm.s_AH);
                W_AB = Vehicle.skewSymmetric(arm.omega);
                M_s = obj.armList{i}.Ms;
              
                w_PA = [0 0 (-1)^(arm.armNumber+1)*arm.motorSpeed]';
                w_PA_dot = [0 0 (-1)^(arm.armNumber+1)*arm.motorAcc]';
                W_PA = Vehicle.skewSymmetric(w_PA);
     
                C3 = -T_BA*arm.inertia*(-W_AB)*T_AB*obj.omega...
                    -T_BA*(W_AB+T_AB*W_BI*T_BA)*arm.inertia*(arm.omega+T_AB*obj.omega)...
                    +T_BA*S_PA*arm.thrust+T_BA*arm.torque+M_s...
                    -T_BA*arm.propInertia*w_PA_dot - T_BA*(W_PA + W_AB + T_AB*W_BI*T_BA)*arm.propInertia*(w_PA + arm.omega + T_AB*obj.omega)...
                    -T_BA*arm.propInertia*(-W_PA)*arm.omega - T_BA*arm.propInertia*(-W_PA + -W_AB)*T_AB*obj.omega;
                C4 = -arm.mass*(((2*W_BI + T_BA*W_AB*T_AB)*T_BA*W_AB*T_AB + W_BI*W_BI)*T_BA*arm.s_AH...
                    +W_BI*W_BI*arm.s_HB-T_BE*g)...
                    +T_BA*arm.thrust;
                b = [b;C3;C4];
                
                C1 = C1-M_s;
                
                % Left side
                A1(:, 9*i+1:9*i+3) = +eye(3)*T_BA;
                A1(:, 9*i+4:9*i+6) = +S_HB*T_BA;
                A2(:, 9*i+4:9*i+6) = +eye(3)*T_BA;
               
                A3(6*i-5:6*i-3, 1:3) = T_BA*(arm.inertia+arm.propInertia)*T_AB;
                A3(6*i-5:6*i-3, 9*i-2:9*i) = T_BA*(arm.inertia+arm.propInertia);
                A3(6*i-5:6*i-3, 9*i+1:9*i+3) = -eye(3)*T_BA;
                A3(6*i-5:6*i-3, 9*i+4:9*i+6) = T_BA*S_AH;

                A3(6*i-2:6*i, 1:3) = -arm.mass*(T_BA*S_AH*T_AB + S_HB);
                A3(6*i-2:6*i, 4:6) = arm.mass*T_BE;
                A3(6*i-2:6*i, 9*i-2:9*i) = -arm.mass*T_BA*S_AH; %Changed
                A3(6*i-2:6*i, 9*i+4:9*i+6) = -eye(3)*T_BA;
            end
            
            %Build A-Matrix:
            A = [A1;A2;A3];
            A_hat = obj.enforceConstraints(A);
            x = A_hat\b;
            x = Vehicle.roundVector(x);
        end
        
        function A_hat = enforceConstraints(obj, A)
            A_hat = [A(:,1:6)];
            for i=1:obj.armNumber
                if (obj.armList{i}.folded)
                    A_hat = [A_hat, A(:,9*i-1), A(:,9*i+1),...
                        A(:, (9*i+3):(9*i+6))];
                else
                    A_hat = [A_hat, A(:, (9*i+1):(9*i+6))];
                end
            end
        end
        
        function obj = run(obj, motorSpeed, dt)
            % Set MotorSpeed;
            for i = 1:obj.armNumber
                obj.armList{i} = obj.armList{i}.setMotorInput(motorSpeed(i), dt);
                obj.armList{i} = obj.armList{i}.calcSpringMoment;
            end
            % Calculate Solution Vector            
            x = obj.calcSolution();
            %assign values
            obj.angAcc = x(1:3);
            obj.transAcc = x(4:6);
            counter_fullyStretched = 0;
            for i = 1:obj.armNumber
                if(obj.armList{i}.folded)
                    % arm is folded somewhere between 90 and 0 degrees
                    obj.armList{i}.phi_ddot = x(6*i+1);
                    obj.fullyStretched = false;
                else
                    % arm is stretched at 0 degrees
                    obj.armList{i}.M2 = x(6*i+2);
                    counter_fullyStretched = counter_fullyStretched+1;

                end
            end
            
            if (counter_fullyStretched == 4)
                obj.fullyStretched = true;
            end
            
            obj = obj.update(dt);
        end
    end
end
  