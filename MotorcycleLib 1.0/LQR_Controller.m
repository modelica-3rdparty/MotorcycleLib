%% LQR_Controller.m
% State Space Controller for the basic bicycle model

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

states = [7,8,3,4];     % Steer Angle, Steer Rate, Lean Angle, Lean Rate

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

%% Poles of the system
poles = eig(Arel)

%% LQR Design
% define the penalizing matrices Q and R
% simplest case -> unity matrices

R = 1;             % single input
% introducing q matrix
q1 = 0;            % weighting factor steer angle
q2 = 0;            % weighting factor steer rate
q3 = 1;            % weighting factor lean angle
q4 = 0;            % weighting factor lean rate
Q = [q1 0 0 0; 0 q2 0 0; 0 0 q3 0; 0 0 0 q4];

F = lqr(Arel, Brel, Q, R)

%% Poles of the closed loop system
p_real = eig(Arel-Brel*F)

