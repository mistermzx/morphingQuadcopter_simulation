%% Script for running the simulation:
%% Clean
clear;
clc;
close all
%% Initialize Simulation
dt = 0.01;
endTime = 10;
simulator = QuadcopterSimulator(dt, endTime);

%% Create Vehicle
m_B = 0.27; %in kg
I_BB = [2.5e-4 0 0; 0 2.5e-4 0; 0 0 2.5e-4];

simulator = simulator.createVehicle(I_BB, m_B);

%% Add Arms
m_A = 0.098;
I_AA = [1.4e-5 0 0; 0 2.7e-4 0; 0 0 2.7e-4];
% all distances in m
dist_ArmHinge = 0.1;
dist_PropHinge = 0.138;
dist_HingeBody = 0.028;

simulator = simulator.addArms(I_AA, m_A, dist_ArmHinge, dist_PropHinge, dist_HingeBody);

%% Add Motor
motSpeedSqrToThrust = 6.4e-6;  % propeller coefficient
motSpeedSqrToTorque = 1.1e-7;
motMaxSpeed = 800; %[rad/s]
% Motor/Propeller Inertia
propInertia = diag([0,0,15e-6]);
motTimeConstant = 0.005;

simulator = simulator.addMotor(motSpeedSqrToThrust, motSpeedSqrToTorque, propInertia, motTimeConstant, motMaxSpeed);

%% Add Spring
% constant force:
springForce = 2;
dist_SH_horizontal = 0.02;
dist_SH_vertical = 0.04;
s_MH = [0.1, 0, -0.02]';

simulator = simulator.addSpring(springForce, dist_SH_horizontal, dist_SH_vertical, s_MH);

%% Define gap
l = 1;
h = 0.2;
w = 0.2;
s_GE = [1; 1; 1];
z_G = [1; 1; 1];
z_G = z_G/norm(z_G);

Gap1 = Gap(s_GE, z_G, l, h, w);
simulator.gap = Gap1;
%simulator.plotGap();

%% Add Controller
%% Position Controller 
disablePositionControl = false;
posCtrlNatFreq = 2;  % rad/s
posCtrlDampingRatio = 0.7;

posController = PositionController(posCtrlNatFreq, posCtrlDampingRatio);
%% Attitude Controller
%time constants for the angle components:
timeConstAngleRP = 0.2;  % [s]
timeConstAngleY  = 1;  % [s]
%gain from angular velocities
timeConstRatesRP = 0.03;  % [s]
timeConstRatesY  = 0.5;   % [s]

attController = AttitudeController(timeConstAngleRP, timeConstAngleY, timeConstRatesRP, timeConstRatesY);

%% Mixer
thrustToTorque = motSpeedSqrToTorque/motSpeedSqrToThrust;
armLength = dist_PropHinge+dist_HingeBody;
mass = m_B+4*m_A; %total mass
dist_ArmHinge = 0.1;
dist_PropHinge = 0.138;
dist_HingeBody = 0.028;
I_AB = I_AA + diag([0 1 1])*m_A*(dist_ArmHinge+dist_HingeBody)^2;
I_AB_90 = diag([I_AB(2,2), I_AB(1,1), I_AB(3,3)]);
inertia = I_BB + 2*I_AB + 2*I_AB_90; %might need to add inertia of arms
mixer = QuadcopterMixer(mass, inertia, armLength, thrustToTorque);
%% Add to simulation
simulator = simulator.addController(posController, attController, mixer);
%% Run Simulation
%% Add noise to the motor speed
simulator.quadcopter.armList{1}.motorMaxSpeed = 1*motMaxSpeed;
%% Add noise to the motorConstants:
simulator.quadcopter.armList{1}.timeConst = 0.005;
%% Different Commands
%% 1) Unfold and stabilize
simulator = simulator.stretchArm();
simulator = simulator.makeDistance([0,0,0]', 3);
%simulator = simulator.foldArm(1);
%% 2) Unfold, stabilize, fold
% simulator = simulator.stretchArm();
% simulator = simulator.makeDistance([0,0,4]', 3);
% simulator = simulator.foldArm(0.5);
% simulator = simulator.hover();
% % 3) Fly through gap
% Uncomment the gapPlot command in plotPosition and animateQuadcopter
% simulator = simulator.flyThroughGap(Gap1);
% simulator = simulator.stretchArm();
% simulator = simulator.hover();

%% Plot Position:
simulator.plotPosition;
%% Plot Arm:
simulator.plotArm(1);
simulator.plotArm(2);
simulator.plotArm(3);
simulator.plotArm(4);
%% Plot Motor Cmds
simulator.plotMotorCmds();
simulator.plotAttitude();
%% Play video of quadcopter
% FPS = 24;
% fileName = 'CurrentVideo';
% movie = simulator.animateQuadcopter (FPS, fileName);


