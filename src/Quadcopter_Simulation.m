%% Quadcopter Simulation:
% Alternative Approach with simplified attitude dynamics for arm
%% Initialize
clear;
clc;
close all
%% Define Parameters
%% Body
m_B = 0.27; %in kg
I_BB = [2.5e-4 0 0; 0 2.5e-4 0; 0 0 2.5e-4];
%% Arm
m_A = 0.098;
I_AA = [1.4e-5 0 0; 0 2.7e-4 0; 0 0 2.7e-4];
s_AH = [0.1, 0, 0]'; % Distance COM A wrt to Hinge
s_PH = [0.138, 0, 0]'; % Distance Propeller P wrt. Hinge
s_HB = 0.028;
% Arm1: 
s_H1B = [s_HB, 0, 0]'; %Distance Hinge to COM Body
% Arm3: 
s_H3B = [-s_HB, 0, 0]';
% Arm 2: 
s_H2B = [0, s_HB, 0]';
% Arm 4:
s_H4B = [0, -s_HB, 0]';


%% Spring
% constant force:
fs = 4;
% Arm1: 
s_S1H1 = [-0.02, 0 -0.04]'; % body_frame
s_M1H1 = [0.1, 0, -0.02]'; % arm-frame
% Arm3: 
s_S3H3 = [0.02, 0 -0.04]';
s_M3H3 = s_M1H1;
% Arm2: 
s_S2H2 = [0 , -0.02 -0.04]';
s_M2H2 = s_M1H1;
% Arm3: 
s_S4H4 = [0 , 0.02 -0.04]';
s_M4H4 = s_M1H1;

%% Motor
motSpeedSqrToThrust = 6.4e-6;  % propeller coefficient
% motSpeedSqrToTorque = 1.1e-7;  % propeller coefficient
motSpeedSqrToTorque = 1.1e-7;
motMaxSpeed = 800; %[rad/s]
% Motor/Propeller Inertia
propInertia = diag([0,0,15e-6]);
motTimeConstant = 0;
%% Simulation Parameters
dt = 0.001;
endTime = 1;
%% Create Object 
arm1 = Arm(I_AA, m_A, s_PH, s_AH, s_H1B,1);
arm1 = arm1.addSpring(s_S1H1, s_M1H1, fs);
arm1 = arm1.addMotor (motSpeedSqrToThrust, motSpeedSqrToTorque, propInertia, motTimeConstant, motMaxSpeed);
arm3 = Arm(I_AA, m_A, s_PH, s_AH, s_H3B, 3);
arm3 = arm3.addMotor (motSpeedSqrToThrust, motSpeedSqrToTorque, propInertia, motTimeConstant, motMaxSpeed);
arm3 = arm3.addSpring(s_S3H3, s_M3H3, fs);

%Scale it up:
arm2 = Arm(I_AA, m_A, s_PH, s_AH, s_H2B, 2);
arm2 = arm2.addSpring(s_S2H2, s_M2H2, fs);
arm2 = arm2.addMotor (motSpeedSqrToThrust, motSpeedSqrToTorque, propInertia, motTimeConstant,motMaxSpeed);
arm4 = Arm(I_AA, m_A, s_PH, s_AH, s_H4B, 4);
arm4 = arm4.addMotor (motSpeedSqrToThrust, motSpeedSqrToTorque, propInertia, motTimeConstant, motMaxSpeed);
arm4 = arm4.addSpring(s_S4H4, s_M4H4, fs);



quadrocopter = Vehicle(I_BB, m_B);
quadrocopter = quadrocopter.addArm(arm1);
quadrocopter = quadrocopter.addArm(arm3);

% Scale it up:
quadrocopter = quadrocopter.addArm(arm2);
quadrocopter = quadrocopter.addArm(arm4);
%% Test Rotation 
% q = Rotation()
% omega = [4,0,0];
% dt = 1;
% q2 = Rotation.from_rotation_vector(omega*dt);
% q = q.compose(q2);
% q.to_euler
%% Test SkewSymmetric
% w = [1;2;3];
% Vehicle.skewSymmetric(w)
%% Run simulation
%% Define Parameters
numSteps = endTime/dt;
index = 1;
t = 0;
time = (dt:dt:endTime)';
%Bookkeeping
posHistory = zeros(numSteps,3);
velHistory = zeros(numSteps,3);
accHistory = zeros (numSteps,3);
attHistory = zeros(numSteps, 3);
angVelHistory = zeros(numSteps, 3);
angAccHistory = zeros(numSteps, 3);
phiHistory = zeros(numSteps, 1);
phiDotHistory = zeros(numSteps, 1);
phiDDotHistory = zeros(numSteps,1);
motThrustHistory = zeros(numSteps,1);
% For Debugging
foldedHistory = zeros(numSteps,1); 
M2History = zeros(numSteps, 1);
MsHistory = zeros(numSteps, 1);
MpHistory = zeros(numSteps, 1);
M2History2 = zeros(numSteps, 1); % Arm 3
MsHistory2 = zeros(numSteps, 1);
MpHistory2 = zeros(numSteps, 1);
phiHistor2 = zeros(numSteps, 1);

%For scaling:
M2History3 = zeros(numSteps, 1); % Arm 2
MsHistory3 = zeros(numSteps, 1);
MpHistory3 = zeros(numSteps, 1);
phiHistor3 = zeros(numSteps, 1);

M2History4 = zeros(numSteps, 1);
MsHistory4 = zeros(numSteps, 1);
MpHistory4 = zeros(numSteps, 1);
phiHistor4 = zeros(numSteps, 1);

%Define Input
motSpeedInput = 1*motMaxSpeed*ones(numSteps,4);
motSpeedInput (numSteps/2:numSteps*0.75, :) = 0;
%% Test Case 1: Unfolding arms with propeller torques
while index<numSteps+1
%     if (quadrocopter.armList{1}.folded==false)
%         break;
%     end
    quadrocopter = quadrocopter.run(motSpeedInput(index, :), dt);
    %Bookkeeping
    foldedHistory(index) = quadrocopter.armList{1}.folded;
    phiHistory(index,:) = quadrocopter.armList{1}.phi;
    phiDotHistory(index,:) = quadrocopter.armList{1}.omega(2);
    phiDDotHistory(index,:) = quadrocopter.armList{1}.phi_ddot;
    posHistory(index, :) = quadrocopter.pos;
    velHistory(index, :) = quadrocopter.vel;
    accHistory(index, :) = quadrocopter.transAcc;
    attHistory(index, :) = quadrocopter.att.to_euler();
    angVelHistory(index, :) = quadrocopter.omega;
    angAccHistory(index, :) = quadrocopter.angAcc;
    motThrustHistory(index, :) = quadrocopter.armList{1}.thrust(3);
    
    M2History(index) = quadrocopter.armList{1}.M2;
    MsHistory(index) = quadrocopter.armList{1}.Ms(2);
    MpHistory(index) = quadrocopter.armList{1}.Mp(2);
    
    phiHistory2(index,:) = quadrocopter.armList{2}.phi;
    M2History2(index) = quadrocopter.armList{2}.M2;
    MsHistory2(index) = quadrocopter.armList{2}.Ms(2);
    MpHistory2(index) = quadrocopter.armList{2}.Mp(2);
    
    phiHistory3(index,:) = quadrocopter.armList{3}.phi;
    M2History3(index) = quadrocopter.armList{3}.M2;
    MsHistory3(index) = quadrocopter.armList{3}.Ms(1);
    MpHistory3(index) = quadrocopter.armList{3}.Mp(2);
    
    phiHistory4(index,:) = quadrocopter.armList{4}.phi;
    M2History4(index) = quadrocopter.armList{4}.M2;
    MsHistory4(index) = quadrocopter.armList{4}.Ms(1);
    MpHistory4(index) = quadrocopter.armList{4}.Mp(2);
    
    index = index+1;
end
%% Plotting
% Body
figure()
subplot(3,1,1)
plot(time(1:index-1), posHistory(1:index-1, :));
legend('x', 'y', 'z');
title('position');
subplot(3,1, 2)
plot(time(1:index-1), velHistory(1:index-1, :));
legend('x', 'y', 'z');
title('velocity');
subplot(3,1, 3)
plot(time(1:index-1), accHistory(1:index-1, :));
title('acceleration');
legend('x', 'y', 'z');

figure()
subplot(3,1,1)
plot(time(1:index-1), attHistory(1:index-1, :));
legend('R', 'P', 'Y');
title('Attitude');
subplot(3,1, 2)
plot(time(1:index-1), angVelHistory(1:index-1, :));
legend('p', 'q', 'r');
title('angVelocity');
subplot(3,1, 3)
plot(time(1:index-1), angAccHistory(1:index-1, :));
title('angAcc');
legend('x', 'y', 'z');

figure()
suptitle('Arm 1')
subplot(4,1,1);
plot(time(1:index-1), phiHistory(1:index-1)*180/pi);
title('\phi');
subplot(4,1,2);
plot(time(1:index-1), M2History(1:index-1));
title('M2')
subplot(4,1,3);
plot(time(1:index-1), MsHistory(1:index-1));
title('Ms(2)')
subplot(4,1,4);
plot(time(1:index-1), MpHistory(1:index-1));
title('Mp')

figure()
suptitle('Arm 3')
subplot(4,1,1);
plot(time(1:index-1), phiHistory2(1:index-1)*180/pi);
title('\phi');
subplot(4,1,2);
plot(time(1:index-1), M2History2(1:index-1));
title('M2')
subplot(4,1,3);
plot(time(1:index-1), MsHistory2(1:index-1));
title('Ms(2)')
subplot(4,1,4);
plot(time(1:index-1), MpHistory2(1:index-1));
title('Mp')


figure()
suptitle('Arm 2')
subplot(4,1,1);
plot(time(1:index-1), phiHistory3(1:index-1)*180/pi);
title('\phi');
subplot(4,1,2);
plot(time(1:index-1), M2History3(1:index-1));
title('M2')
subplot(4,1,3);
plot(time(1:index-1), MsHistory3(1:index-1));
title('Ms(1)')
subplot(4,1,4);
plot(time(1:index-1), MpHistory3(1:index-1));
title('Mp')

figure()
suptitle('Arm 4')
subplot(4,1,1);
plot(time(1:index-1), phiHistory4(1:index-1)*180/pi);
title('\phi');
subplot(4,1,2);
plot(time(1:index-1), M2History4(1:index-1));
title('M2')
subplot(4,1,3);
plot(time(1:index-1), MsHistory4(1:index-1));
title('Ms(1)')
subplot(4,1,4);
plot(time(1:index-1), MpHistory4(1:index-1));
title('Mp')






