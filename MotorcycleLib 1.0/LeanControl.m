%% LeanControl.m
% design of a lean controller for the basic motorcycle model

clc;

%% load dslin.math and show the names of the states, inputs and outputs
load dslin.mat
xuyName

%% States
% 1.  leanAngle
% 2.  leanRate

%% Input
% u1: Steering torque

[A,B,C,D]=tloadlin('dslin.mat');
%%
% Define the relevant A Matrix depending on the states needed for the
% controller
states = [1, 2];

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
disp(' ')
disp('Compute the poles (p) of the vehicle:')
p = eig(Arel)

p1 = -4;
p2 = -5;

disp('Controller Matrix F:');
F = place(Arel, Brel, [p1, p2])

% check the pole locations
p_real = eig(Arel-Brel*F)

