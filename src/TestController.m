%% Test Controller
clear;
clc;
%% Attitude Controller
%time constants for the angle components:
timeConstAngleRP = 0.1;  % [s]
timeConstAngleY  = 1.0;  % [s]
%gain from angular velocities
timeConstRatesRP = 0.05;  % [s]
timeConstRatesY  = 0.5;   % [s]

attController = AttitudeController(timeConstAngleRP, timeConstAngleY, timeConstRatesRP, timeConstRatesY);

thrustNormDes = [0, 0, 9.81]';
att = Rotation.from_euler_RPY([pi/4, 0, pi/4]);
att.q
omega = [0 0 0]';
angAccDes = attController.get_angular_acceleration(thrustNormDes, att,omega);
angAccDes

%% Test mixer
armLength = 0.17;
mass = 0.5;
thrustToTorque = 1.1e-7/6.4e-6;
inertia = [2.7e-3 0 0;...
    0 2.7e-3 0;...
    0 0 5.2e-3];

mixer = QuadcopterMixer(mass, inertia, armLength, thrustToTorque);
motForceCmds = mixer.get_motor_force_cmd(thrustNormDes, angAccDes);
motForceCmds