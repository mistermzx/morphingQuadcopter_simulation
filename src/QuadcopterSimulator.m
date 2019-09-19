classdef QuadcopterSimulator
    properties
        quadcopter; %Vehicle object
        
        %Controller (all Objects)
        positionController;
        attitudeController;
        mixer;
        
        dt; %timstep
        T; %simulationTime
        index;
        numSteps;
        timeVec;
        
        posHistory;
        velHistory;
        accHistory;
        
        attHistory;
        angVelHistory;
        angAccHistory;
        
        armAngleHistory;
        hingeMomentHistory; %M2
        springMomentHistory; %Ms
        
        motThrustHistory;
        
        %Gap
        gap;
        initialPos;
        
        %for drawing
        s_HEHistory; %CellArray of Hinge Points in E-frame
        s_PEHistory; %CellArray of Propeller Points in E-frame
    end
    
    methods
        %%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
        % Construct Vehicle %
        %%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%

        function obj = QuadcopterSimulator(timeStep,simTime)
            % Initializes the Simulation
            % with a timeStep and simulationTime
            obj.dt = timeStep;
            obj.T = simTime;
            obj.index = 1;
            numSteps = simTime/timeStep;
            obj.numSteps = numSteps;
            obj.timeVec = (0:timeStep:simTime)';
            
            obj.posHistory = zeros(numSteps,3);
            obj.velHistory = zeros(numSteps,3);
            obj.accHistory = zeros (numSteps,3);
            obj.attHistory = zeros(numSteps, 3);
            obj.angVelHistory = zeros(numSteps, 3);
            obj.angAccHistory = zeros(numSteps, 3);

            obj.armAngleHistory = zeros(numSteps, 4);
            obj.hingeMomentHistory = zeros(numSteps, 4); %M2
            obj.springMomentHistory = zeros(numSteps, 4); %Ms

            obj.motThrustHistory = zeros(numSteps, 4);
            
            obj.s_HEHistory = cell(1,numSteps);
            obj.s_PEHistory = cell(1, numSteps);
            
            obj.initialPos = zeros(3,1);
            obj.gap = false;
        end
        
        function obj = createVehicle (obj, inertiaMatrix, mass)
            % Creates a vehicle object and assigns it to the simulation
            % Input: inertiaMatrix, mass 
            obj.quadcopter = Vehicle(inertiaMatrix, mass);
        end
        
        function obj = addArms (obj, inertiaMatrix, mass,...
                dist_ArmHinge, dist_PropHinge, dist_HingeBody)
            % Adds all four arms (symmetrically) to the Quadcopter
            % all distances will be passed as a scalar
           
            s_HB = [[dist_HingeBody, 0, 0]' [0, dist_HingeBody, 0]' ...
                [-dist_HingeBody, 0, 0]' [0, -dist_HingeBody, 0]']; %expressed in the body frame
            s_AH = [dist_ArmHinge, 0, 0]'; % expressed in the A-frame
            s_PH = [dist_PropHinge, 0, 0]'; % expressed in the A-frame
            
            for i = 1:4
                currArm = Arm(inertiaMatrix, mass, s_PH, s_AH, s_HB(:, i), i);
                obj.quadcopter = obj.quadcopter.addArm(currArm);
            end 
        end
        
        function obj = addMotor (obj, motSpeedSqrToThrust, motSpeedSqrToTorque, propInertia, motTimeConst, motMaxSpeed)
            % Adds motors to the arms
            
            for i = 1:obj.quadcopter.armNumber
                obj.quadcopter.armList{i} = obj.quadcopter.armList{i}.addMotor(motSpeedSqrToThrust, motSpeedSqrToTorque, propInertia, motTimeConst, motMaxSpeed);
            end
        end
        
        function obj = addSpring(obj, springForce, dist_SH_horizontal, dist_SH_vertical, s_MH)
            % Adds a constant force spring to the arms
            % Mounted between S (on body) and M (on arm)
            % all distances are scalars and positive
            % s_MH is a vector and expressed in the A-frame
            
            s_SH = [[-dist_SH_horizontal, 0, -dist_SH_vertical]' [0, -dist_SH_horizontal, -dist_SH_vertical]'...
                [dist_SH_horizontal, 0, -dist_SH_vertical]' [0, dist_SH_horizontal, -dist_SH_vertical]']; %expressed in the B-CS
            
            for i = 1:obj.quadcopter.armNumber
                obj.quadcopter.armList{i} = obj.quadcopter.armList{i}.addSpring(s_SH(:,i), s_MH, springForce);
            end
            
            % Calculate minimum thrust needed
            obj.quadcopter.motorThrust_min = 0;
            for i = 1:4
                Ms = obj.quadcopter.armList{i}.calcSpringMoment2(0);
                arm = obj.quadcopter.armList{i};
                minThrust = (norm(Ms)+arm.mass*9.81*norm(arm.s_AH))/(norm(obj.quadcopter.armList{i}.s_PH));
                if (obj.quadcopter.motorThrust_min<minThrust);
                    obj.quadcopter.motorThrust_min = minThrust;
                end
            end
            disp('The minimum motor thrust (N) and moment (Nm) to keep the arms stretched:');
            minThrust = obj.quadcopter.motorThrust_min
            minTorque = obj.quadcopter.motorThrust_min*norm(obj.quadcopter.armList{1}.s_PH)
            obj.quadcopter.hoverThrust = 9.81*(obj.quadcopter.mass+4*obj.quadcopter.armList{1}.mass)/4;
            hoverThrust = obj.quadcopter.hoverThrust
            
        end
        
        function obj = addController(obj, posController, attController, quad_mixer)
            % adds the controller/mixer objects to the simulation
            obj.positionController = posController;
            obj.attitudeController = attController;
            obj.mixer = quad_mixer;
        end
        
        function obj = addGap (obj, gapCenter, gapDirection, gapLength, gapHeight, gapWidth)
           obj.gap = Gap(gapCenter, gapDirection, gapLength, gapHeight, gapWidth); 
        end
        
        %%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
        % Run Simulation %
        %%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
        function obj = saveStuffForDrawing(obj)
            % Saving S_PE and s_HE vector for drawing the quadcopter later
            
            % Some constants
            dist_HingeBody = obj.quadcopter.armList{1}.s_HB(1);
            s_PH = obj.quadcopter.armList{1}.s_PH; %Distance Propeller Hinge in A-frame
            s_HB = [[dist_HingeBody, 0, 0]' [0, dist_HingeBody, 0]' ...
                [-dist_HingeBody, 0, 0]' [0, -dist_HingeBody, 0]']; % Distance Hinge Body in B-frame
            
            T_BE = obj.quadcopter.att.to_trans_matrix();
            T_EB = T_BE';
            s_PE = [];
            s_HE = [];
            s_BE = obj.quadcopter.pos;
            for i = 1:4 
                s_HE = [s_HE, s_BE+T_EB*s_HB(:,i)];
                
                T_BH = obj.quadcopter.armList{i}.T_BH;
                e_2 = [0, 1, 0]';
                T_HA = Rotation.fromAxisAngle(e_2, -obj.quadcopter.armList{i}.phi);
                armOrient = T_HA.compose(T_BH);
                T_BA = armOrient.to_trans_matrix();
                
                s_PE = [s_PE, T_EB*T_BA*s_PH+T_EB*s_HB(:,i)+s_BE];
            end
            obj.s_PEHistory{obj.index} = s_PE;
            obj.s_HEHistory{obj.index} = s_HE;
        end
        function obj = saveData(obj)
            % Helper function to save the current simulation data into the
            % arrays
            index = obj.index;
            quadrocopter = obj.quadcopter;
            
            obj.posHistory(index, :) = quadrocopter.pos;
            obj.velHistory(index, :) = quadrocopter.vel;
            obj.accHistory(index, :) = quadrocopter.transAcc;
            obj.attHistory(index, :) = quadrocopter.att.to_euler().*180/pi;
            obj.angVelHistory(index, :) = quadrocopter.omega;
            obj.angAccHistory(index, :) = quadrocopter.angAcc;
            
            for i = 1:4
                obj.armAngleHistory (index, i) = quadrocopter.armList{i}.phi;
                obj.hingeMomentHistory (index, i) = quadrocopter.armList{i}.M2;
                obj.springMomentHistory (index, i) = quadrocopter.armList{i}.Ms(2)+quadrocopter.armList{i}.Ms(1);
                obj.motThrustHistory(index, i) = quadrocopter.armList{i}.thrust(3);
            end
            
            obj = obj.saveStuffForDrawing();
        end
        
        function obj = stretchArm(obj)
            % stretches all arms, by setting all motor inputs to max
            motSpeedInput = [];
            for i = 1:4
                motSpeedInput = [motSpeedInput, obj.quadcopter.armList{i}.motorMaxSpeed*ones(obj.numSteps,1)]; %omega of motor
            end
            
            while (obj.index<obj.numSteps+1)
                if (obj.quadcopter.fullyStretched)
                    break;
                end
                obj.quadcopter = obj.quadcopter.run(motSpeedInput(obj.index, :), obj.dt);
                obj = obj.saveData();
                obj.index = obj.index + 1;
            end
        end
        
        function obj = foldArm(obj, T)
            % folds all arms, by setting all motor inputs to zero and runs
            % the simulation for time T
            initTime = obj.index*obj.dt;
            motSpeedInput = 0*ones(obj.numSteps,4); %omega of motor
            while (obj.index<obj.numSteps+1)
                if (obj.index*obj.dt>initTime+T)
                    break;
                end
                obj.quadcopter = obj.quadcopter.run(motSpeedInput(obj.index, :), obj.dt);
                obj = obj.saveData();
                obj.index = obj.index + 1;
            end
            
        end
        
        function motSpeedCmds = get_controller_cmds(obj, desPos)
            % Calculates the controller output 
            %accDes = obj.positionController.get_acceleration_command(desPos, obj.quadcopter.pos, obj.quadcopter.vel);
            accDes = 0;
            thrustNormDes = accDes + [0, 0, 9.81]';
            angAccDes = obj.attitudeController.get_angular_acceleration(thrustNormDes, obj.quadcopter.att,obj.quadcopter.omega);
            motForceCmds = obj.mixer.get_motor_force_cmd(thrustNormDes, angAccDes);
            if (motForceCmds<obj.quadcopter.motorThrust_min)
               motForceCmds = obj.quadcopter.motorThrust_min;
            end
         
            speedSqrToThrust = obj.quadcopter.armList{1}.thrustConstant;
            motSpeedCmds = (motForceCmds./speedSqrToThrust).^0.5;
        end
        
        function obj = hover(obj)
            % hovers quadcopter for time T
            initTime = obj.index*obj.dt;
            while (obj.index<obj.numSteps+1)
                accDes = obj.positionController.get_acceleration_command(zeros(3,1), obj.quadcopter.pos, obj.quadcopter.vel);
                thrustNormDes = accDes+[0, 0, 9.81]';
                angAccDes = obj.attitudeController.get_angular_acceleration(thrustNormDes, obj.quadcopter.att, obj.quadcopter.omega);
                motForceCmds = obj.mixer.get_motor_force_cmd(thrustNormDes, angAccDes);
                % Input saturations: lower bound
                safety = 1;
                for i = 1:4
                    if (motForceCmds(i)<safety*obj.quadcopter.motorThrust_min)
                       motForceCmds(i) = safety*obj.quadcopter.motorThrust_min;
                    end
                end
                speedSqrToThrust = obj.quadcopter.armList{1}.thrustConstant;
                motSpeedCmds = (motForceCmds./speedSqrToThrust).^0.5;
                
                obj.quadcopter = obj.quadcopter.run(motSpeedCmds, obj.dt);
                obj = obj.saveData();
                obj.index = obj.index + 1;
            end
        end
        
        function obj = makeDistance(obj, desPos, T)
            % Uses quadcopter standard controller to fly to a specified
            % point (desPos) (vector)
            initTime = obj.index*obj.dt;
            while (obj.index<obj.numSteps+1)
                if (obj.index*obj.dt-initTime>T)
                    break; 
                end
                accDes = obj.positionController.get_acceleration_command(desPos, obj.quadcopter.pos, obj.quadcopter.vel);
                thrustNormDes = accDes+[0, 0, 9.81]';
                angAccDes = obj.attitudeController.get_angular_acceleration(thrustNormDes, obj.quadcopter.att, obj.quadcopter.omega);
                motForceCmds = obj.mixer.get_motor_force_cmd(thrustNormDes, angAccDes);
                % Input saturations: lower bound
                safety = 1;
                for i = 1:4
                    if (motForceCmds(i)<safety*obj.quadcopter.motorThrust_min)
                       motForceCmds(i) = safety*obj.quadcopter.motorThrust_min;
                    end
                end
                speedSqrToThrust = obj.quadcopter.armList{1}.thrustConstant;
                motSpeedCmds = (motForceCmds./speedSqrToThrust).^0.5;
                
                obj.quadcopter = obj.quadcopter.run(motSpeedCmds, obj.dt);
                obj = obj.saveData();
                obj.index = obj.index + 1;
            end
        end
        
        function obj = setMotorInput(obj, motorInput, time)
            % sets motor input to motorInput for a speficied time
            initTime = obj.index*obj.dt;
            index = 1;
            while (obj.index<obj.numSteps+1)
                if (obj.index*obj.dt>initTime+time)
                    break;
                end
                obj.quadcopter = obj.quadcopter.run(motorInput(index, :), obj.dt);
                obj = obj.saveData();
                index = index+1;
                obj.index = obj.index + 1;
            end
        end
        
        function obj = setAttitude (obj, velDes)
            % Sets attitude to match the desired Velocity
            
            yaw = atan2(velDes(2), velDes(1));
            h = velDes(1)^2+velDes(2)^2;
            pitch = atan2(h, velDes(3));
            roll = 0;
            obj.quadcopter.att = Rotation.from_euler_RPY([roll, pitch, yaw]);
        end
        
        function obj = setState(obj, posDes, velDes)
            % Sets the position and velocity to the desired values.
            % Arbitrarily, without controller and checking feasibility
            % Input: posDes, velDes in earth frame
            
            obj = obj.setAttitude(velDes);
            T_BE = obj.quadcopter.att.to_trans_matrix();
            obj.quadcopter.pos = posDes;
            obj.quadcopter.vel = velDes;
            % Stretch all arms:
            for i = 1:4
                obj.quadcopter.armList{i}.folded = false;
                obj.quadcopter.armList{i}.phi= 0;
                obj.quadcopter.armList{i}.motorSpeed = 251.8335; %Hover Speed
            end
            % Debugging
            disp(T_BE*velDes);

            %obj = obj.saveData();
           % obj.index = obj.index+1;
        end
        
        function obj = flyThroughGap(obj, gapObject)
            % flies through the defined gapObject
            obj.gap = gapObject;
            [posDes, velDes, T_f] = gapObject.generate_trajectory()
            obj.initialPos = posDes;
            obj = obj.setState(posDes, velDes);
            %obj.quadcopter.omega = [0, 2, 0]';
            % Set yaw rotation
            obj.quadcopter.omega(3) = 0;
            obj = obj.foldArm(T_f);
        end
        %%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
        % Plotting Stuff %
        %%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
        function plotArm (obj, armNumber)
            time = obj.timeVec;
            index = obj.index;
            figure()
            suptitle(['Arm ', num2str(armNumber)]);
            subplot(4,1,1);
            plot(time(1:index-1), obj.armAngleHistory(1:index-1, armNumber)*180/pi);
            title('\phi');
            subplot(4,1,2);
            plot(time(1:index-1),obj.hingeMomentHistory(1:index-1, armNumber));
            title('HingeMoment in arm-frame')
            subplot(4,1,3);
            plot(time(1:index-1), obj.springMomentHistory(1:index-1, armNumber));
            title('SpringMoment in body-frame')
            subplot(4,1,4);
            plot(time(1:index-1),obj.motThrustHistory(1:index-1, armNumber));
            title('Motor Thrust')
            saveas( gcf, ['Arm ', num2str(armNumber)], 'jpg' );

        end
        
        function plotGap(obj)
            % Plots the gap 
            % TODO: insert check, if there is an obstacle
            s_GE = obj.gap.s_GE;
            z_G = obj.gap.z_G;
            l = obj.gap.l;
            h = obj.gap.h;
            w = obj.gap.w;
            T_GE = obj.gap.T_GE;
            T_EG = T_GE';
            
            plot3(s_GE(1), s_GE(2), s_GE(3), 'b.');
            plot3([s_GE(1), s_GE(1)+z_G(1)], ...
      [s_GE(2), s_GE(2)+z_G(2)], ...
      [s_GE(3), s_GE(3)+z_G(3)], 'b');
            % Generate corners of box and plot box
            % Bottom
            corners = [-h h h -h -h;
                       -w -w w w -w;
                       -l -l -l -l -l];
            % Sides
            corners = [corners, ...
                      [-h  h  h  h h  h h -h -h -h -h;
                       -w -w -w -w w  w w  w  w w -w;
                       l   l -l  l l -l l  l -l l l]];
            % Rotate to desired frame
            corners = T_EG*corners./2 + s_GE;
            plot3(corners(1,:), corners(2,:), corners(3,:), '-');
        end
        
        function plotPosition (obj)
          
            index = obj.index;
            time = obj.timeVec;
            
            % 3D Plot
            figure()
            hold on;
            grid on;
            plot3(obj.posHistory(1,1),obj.posHistory(1,2),obj.posHistory(1,3), 'ro');
            plot3(obj.posHistory(1:index-1,1),obj.posHistory(1:index-1,2),obj.posHistory(1:index-1,3));
            %plot3(obj.initialPos(1),obj.initialPos(2),obj.initialPos(3), 'rx');
            plot3(obj.posHistory(index-1,1),obj.posHistory(index-1,2),obj.posHistory(index-1,3), 'square');

            %obj.plotGap();
            title('Quadcopter Position');
            xlabel('x-Position');
            ylabel('y-Position');
            zlabel('z-Position');
            view(3);
            legend('Starting point', 'Trajectory', 'End Point');
            saveas( gcf, 'Position', 'jpg' );

            figure()
            subplot(3,1,1)
            plot(time(1:index-1), obj.posHistory(1:index-1, :));
            legend('x', 'y', 'z');
            title('position');
            subplot(3,1, 2)
            plot(time(1:index-1), obj.velHistory(1:index-1, :));
            legend('x', 'y', 'z');
            title('velocity');
            subplot(3,1, 3)
            plot(time(1:index-1), obj.accHistory(1:index-1, :));
            title('acceleration');
            legend('x', 'y', 'z');
            saveas( gcf, 'Translational', 'jpg' );

        end
        
        function plotAttitude(obj)
            index = obj.index;
            time = obj.timeVec;
            
            figure()
            subplot(3,1,1)
            plot(time(1:index-1), obj.attHistory(1:index-1, :));
            legend('R', 'P', 'Y');
            title('Attitude');
            subplot(3,1, 2)
            plot(time(1:index-1), obj.angVelHistory(1:index-1, :));
            legend('p', 'q', 'r');
            title('angVelocity');
            subplot(3,1, 3)
            plot(time(1:index-1), obj.angAccHistory(1:index-1, :));
            title('angAcc');
            legend('x', 'y', 'z');
            saveas( gcf, 'Attitude', 'jpg' );
        end
        
        function plotMotorCmds (obj)
            time = obj.timeVec;
            index = obj.index;
            
            minLine = obj.quadcopter.motorThrust_min*ones(1, obj.numSteps);
            maxLine = obj.quadcopter.armList{2}.motorMaxSpeed^2*obj.quadcopter.armList{2}.thrustConstant*ones(1,obj.numSteps);
            
            figure()
            hold on;
            for i = 1:4
                plot(time(1:index-1), obj.motThrustHistory(1:index-1, i));
            end
            
            plot(time(1:index-1), minLine(1:index-1));
            plot(time(1:index-1), maxLine(1:index-1));
            
            legend('Arm1', 'Arm2', 'Arm3', 'Arm4', 'MinThrust', 'MaxThrust');
            title('Motor Thrust');
            xlabel('time in s')
            ylabel('Thrust in N')
            saveas( gcf, 'MotorThrust', 'jpg' );
        end
        
        function drawQuadcopter(obj, index)
            % draws a sketch of the current state of the quadcopter in a 3D
            % plot

            quadPos = obj.posHistory(index, :)'; %in E-frame

            clf;
            hold on;
            grid on;
            plot3(quadPos(1), quadPos(2), quadPos(3), 'xr');
            for i = 1:4
                % Draw body
                s_HE = obj.s_HEHistory{index}(:,i);
                plot3([quadPos(1), s_HE(1)], [quadPos(2), s_HE(2)], [quadPos(3), s_HE(3)], 'b');
                plot3(s_HE(1), s_HE(2), s_HE(3), 'or');
                
                %Draw arm
                s_PE = obj.s_PEHistory{index}(:,i);
                plot3([s_HE(1), s_PE(1)], [s_HE(2), s_PE(2)], [s_HE(3), s_PE(3)], 'k');
                plot3(s_PE(1), s_PE(2), s_PE(3), 'sr');                
            end
            title(['Time: ', num2str(index*obj.dt), 's']);
            xlabel('x-Position');
            ylabel('y-Position');
            zlabel('z-Position');
            pbaspect([1 1 1])

            
            view(-79, 10);
            
        end
        
        function movies = animateQuadcopter(obj, Fps, fileName)
            % Plays anmitated video of quadcopter
            nFrames = obj.index-1;
            F(nFrames) = struct('cdata',[],'colormap',[]);
            disp('Creating and recording frames...');
            
            % for window scaling
            tol = 0.2;
            xmin = min(obj.posHistory(:,1))-tol;
            xmax = max(obj.posHistory(:,1))+tol;
            ymin = min(obj.posHistory(:,2))-tol;
            ymax = max(obj.posHistory(:,2))+tol;
            zmin = min(obj.posHistory(:,3))-tol;
            zmax = max(obj.posHistory(:,3))+tol;

           
            
            l = max([abs(xmax-xmin), abs(ymax-ymin), abs(zmax-zmin)]);
            vidObj = VideoWriter(strcat(fileName, '.avi'),'Motion JPEG AVI');
            open(vidObj);
            
            for j = 1:nFrames
                obj.drawQuadcopter(j);
                %obj.plotGap();
                axis([xmin, xmin+l, ymin, ymin+l, zmin, zmin+l])
                drawnow
                F(j) = getframe;
                writeVideo(vidObj, getframe(gcf));
            end
            %Play video
 
            %x = input(message);
%             nPlay = 1;
%             movie(F,nPlay, Fps);
              movies = F;
        end
       

    end
        
end