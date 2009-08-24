%% StateSpaceController.m
% State Space Controller for the basic motorcycle model

clc;

%% load dslin.math and show the names of the states, inputs and outputs
load dslin.mat
xuyName

%% States
% 1.  FWRevolute.phi
% 2.  wheel_sensed.xA
% 3.  wheel_sensed.xB
% 4.  wheel_sensed.phi[1]
% 5.  wheel_sensed.phi[2]
% 6.  wheel_sensed.phi[3]
% 7.  wheel_sensed.phi_d[1]
% 8.  wheel_sensed.phi_d[2]
% 9.  wheel_sensed.phi_d[3]
% 10. Steering.phi
% 11. derivative.x

%% Inputs
% u1: Steering torque

%% Outputs (measured)
% x1: lateral Position
% x2: lateral Velocity
% x3: phiLean

[A,B,C,D]=tloadlin('dslin.mat');
%%
% Define the relevant A Matrix depending on the states needed for the
% controller

states = [12, 11, 5, 8];
%states = [3, 11, 5, 8];
%states = [2, 3, 5, 7];

Arel = A(states, states);
Brel = B(states,:);
Crel = C(:,states);

vehicle = ss(Arel, Brel, Crel, D);

%% Is the System controlable?
% Compute the controlability matrix
Co = ctrb(Arel, Brel);

% If the rank of the matrix is equal to the system's states then the system
% is controlable

rang = rank(Co);
[m, n] = size(Arel);
if (rang < n)
    disp('r < n --> System is not controlable!');
else
    disp('System controlable!');
end

%% Controller design
% to design a simple state space controller we first compute the root locus
% of the system --> does not work - only for SISO Systems
% compute the pole-zero map of the system

% pz = pzmap(vehicle)
disp(' ')
disp('Compute the poles (p) of the vehicle:')
p = pole(vehicle)

% pole pla1ement according to pole-zero map
p1 = 0;  % position control
p2 = 0.8;  % position control
%p2 = -99.8
%p2 = -0.001;
p3 = 1.5;    % lean control
p4 = -3.8;    % lean control

disp('Controller Matrix F:');
F = place(Arel, Brel, [p1, p2, p3, p4])

