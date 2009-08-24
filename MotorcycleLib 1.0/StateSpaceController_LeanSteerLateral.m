%% StateSpaceController_LeanSteerLateral.m
% State Space Controller for the basic motorcycle model
clc;

%% load dslin.math and show the names of the states, inputs and outputs
load dslin.mat;
xuyName;

%% Input
% u1: Steering torque

[A,B,C,D]=tloadlin('dslin.mat');
%%
% Define the relevant A Matrix depending on the states needed for the
% controller

% states: 5 = steer angle, 6 = steer rate, 1 = lean angle, 2 = lean rate,
%         11 = lateral position, 10 = lateral rate
states = [5, 6, 1, 2, 11, 10];
%states = [11, 10, 5, 6, 1, 2];

Arel = A(states, states);
Brel = B(states,:);

%% Controller design
% to design a simple state space controller we first compute the root locus
% of the system --> does not work - only for SISO Systems
% compute the pole-zero map of the system
disp(' ')
disp('Compute the poles (p) of the vehicle:')
p = eig(Arel);

% pole placement according to pole-zero map
offset = 5;
p1 = p(1) - 1;
p2 = p(2) - 1.0000001;
p3 = p(3) - offset;
p4 = p(4) - offset;
p5 = p(5) - offset;
p6 = p(6) - offset;

poles = [p1, p2, p3, p4, p5, p6];

disp('Controller Matrix F:');
F = place(Arel, Brel, poles)

% LQR Design
R = 1;
Q = eye(6);
Q(1,1) = 0;
Q(2,2) = 0;
Q(3,3) = 1;
Q(4,4) = 0;
Q(5,5) = 10;
Q(6,6) = 0;

F2 = lqr(Arel, Brel, Q, R)

% check the pole location
p_real = eig(Arel-Brel*F);





