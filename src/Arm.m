classdef Arm
    properties 
        mass;
        s_PH;
        s_AH;
        s_HB; %Note: expressed in B CS
        inertia;
        phi;
        phi_ddot;
        omega;
        orientation; %T_BA
        T_BH; 
        % Spring
        folded = true; 
        spring;
        %Motor Stuff
        thrustConstant;
        torqueConstant;
        thrust;
        torque;
        %Propelller;
        propInertia;
        motorSpeed;
        motorAcc;
        timeConst;
        motorMaxSpeed;
        %for propeller torque
        armNumber;
        % moments at hinge
        M2 = 0;
        Ms; %wrt B-CS
        Mp; %for debugging only
    end
    
    methods 
        function obj = Arm(inertiaMatrix, mass, s_PH, s_AH, s_HB, no)
            obj.mass = mass;
            obj.inertia = inertiaMatrix;
            obj.s_PH = s_PH;
            obj.s_AH = s_AH;
            obj.s_HB = s_HB;
            obj.phi = pi/2;
            obj.omega = [0, 0, 0]';
            obj.phi_ddot = 0; 
            
            
            %Orientation
            theta = atan2(s_HB(2), s_HB(1)); % Yaw rotation from body to hinge
            e_3 = [0; 0; 1];
            obj.T_BH = Rotation.fromAxisAngle(e_3, -theta);
            e_2 = [0; 1; 0];
            T_HA = Rotation.fromAxisAngle(e_2, -obj.phi);
            obj.orientation = T_HA.compose(obj.T_BH);
            
            %number
            obj.armNumber = no;
        end
        % adds Motor
        function obj = addMotor(obj, thrustCon, torqueCon, pInertia, tau, maxSpeed)
            %Motor Stuff
            obj.thrustConstant = thrustCon;
            obj.torqueConstant = torqueCon;
            obj.thrust = [0, 0, 0]';
            obj.torque = [0, 0, 0]';
            % Propeller
            obj.motorSpeed = 0;
            obj.motorAcc = 0;
            obj.propInertia = pInertia;
            obj.timeConst = tau;
            obj.motorMaxSpeed = maxSpeed;
        end
        
        function obj = addSpring(obj, s_SH, s_MH, fs)
            obj.spring = cfSpring(s_SH, s_MH, fs);
        end
        
        %Returns spring moment wrt. H in the A-CS
        function obj = calcSpringMoment(obj)
            T_BA = obj.orientation.to_trans_matrix();
            T_AB = T_BA';
            s_MS = obj.spring.s_MH-T_AB*obj.spring.s_SH; %in A frame
            obj.spring.T_AS = rotationVectorToMatrix([0;-1;0].*atan2(s_MS(3), s_MS(1)))';
            F_s = obj.spring.T_AS*[-obj.spring.fs;0;0];
            obj.Ms = T_BA*cross(obj.spring.s_MH, F_s);
        end     
        
        function Ms = calcSpringMoment2(obj, phi)
            e_2 = [0; 1; 0];
            T_HA = Rotation.fromAxisAngle(e_2, phi);
            orientation = T_HA.compose(obj.T_BH);
            % Calculates springMoment for an arbitrary phi and returns it
            T_BA = orientation.to_trans_matrix();
            T_AB = T_BA';
            s_MS = obj.spring.s_MH-T_AB*obj.spring.s_SH; %in A frame
            T_AS = rotationVectorToMatrix([0;-1;0].*atan2(s_MS(3), s_MS(1)))';
            F_s = T_AS*[-obj.spring.fs;0;0];
            Ms = T_BA*cross(obj.spring.s_MH, F_s);
        end      
        
        function obj = updateOrientation(obj)
            e_2 = [0; 1; 0];
            T_HA = Rotation.fromAxisAngle(e_2, -obj.phi);
            obj.orientation = T_HA.compose(obj.T_BH);
        end
        
        function obj = updateArm(obj, dt)
             if (obj.folded)
                % Arm is folded between 0 and 90 degrees
                phi_dot = obj.omega(2);
                obj.phi = obj.phi + phi_dot*dt;
                obj.omega(2) = obj.omega(2) + obj.phi_ddot*dt;
                if (obj.phi < 0)
                    % Arm is stretched
                    obj.omega(2) = 0;
                    obj.phi = 0;
                    %obj.phi_ddot = 0;
                    obj.folded = false;
                end
                if (obj.phi>pi/2)
                    % Arm is completely folded
                    obj.omega(2) = 0;
                    obj.phi = pi/2;
                    %obj.phi_ddot = 0;
                    obj.folded = true;
                end
             else
                 % arm is stretched at 0 degrees
                 if (obj.M2<0)
                    % arm is stretched, but wants to fold
                    obj.folded = true;
                    obj.M2 = 0;
                 end
             end
             obj = obj.updateOrientation();
        end
        
        function obj = setMotorInput(obj, motorSpeedDes, dt)
            if (obj.timeConst == 0)
                c = 0;
            else
                c = exp(-dt/obj.timeConst);
            end
            currSpeed = obj.motorSpeed;
            obj.motorSpeed = c*obj.motorSpeed+(1-c)*motorSpeedDes;
            if (obj.motorSpeed>obj.motorMaxSpeed)
                obj.motorSpeed = obj.motorMaxSpeed;
            end
            obj.motorAcc = (obj.motorSpeed-currSpeed)/dt;
      
            obj.thrust(3) = obj.thrustConstant*obj.motorSpeed^2;
            obj.Mp = cross(obj.s_PH, obj.thrust); %for debugging
            obj.torque(3) = (-1)^obj.armNumber*obj.torqueConstant*obj.motorSpeed^2;
        end
            
            
    end
    
end