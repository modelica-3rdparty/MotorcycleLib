%% Bicycle_StateSpaceController_LeanSteer_RiderLean.m
% State Space Controller for a 4 d.o.f bicycle model
% the output of this function is a state feedback matrix calculated in two
% different ways:
%   - Pole placement:   place()
%   - Optimal control:  lqr()

clc;

%% load dslin.math and show the names of the states, inputs and outputs
load dslin.mat;
xuyName;

%% Input
% u1: Steering torque
% u2: Rider lean torque

[A,B,C,D]=tloadlin('dslin.mat');
%%
% Define the relevant A Matrix depending on the states needed for the
% controller

% state selection
% states = [steer angle, steer rate
%           lean angle, lean rate,
%           rider lean angle, rider lean rate];

states = [7, 8, 3, 4, 9, 10];
Arel = A(states, states);
Brel = B(states,:);

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
p = eig(Arel)

% pole pla1ement according to pole-zero map

offset = 2;
offset_rider = 4;
p1 = p(1) - offset;
p2 = p(2) - offset_rider;
p3 = p(3) - offset;
p4 = p(4) - offset;
p5 = p(5) - offset;
p6 = p(6) - offset_rider;

poles = [p1, p2, p3, p4, p5, p6];

disp('Controller Matrix F:');
F = place2(Arel, Brel, poles)

% LQR Design
R = [0.1 0; 0 0.1];
% introducing Q matrix
q1 = 0;
q2 = 0;
q3 = 100;
q4 = 0;
q5 = 100;
q6 = 0;

Q = diag([q1, q2, q3, q4, q5, q6]);

F_lqr = lqr(Arel, Brel, Q, R)

% check the pole location
p_real = eig(Arel-Brel*F)

