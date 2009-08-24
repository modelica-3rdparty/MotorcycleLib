%% StateSpaceController.m
% State Space Controller for the basic motorcycle model

clc;

%% load dslin.math and show the names of the states, inputs and outputs
load dslin.mat;
xuyName;

[A,B,C,D]=tloadlin('dslin.mat');

%% State variables
%states = [7,8,3,4];     % states basic bicycle
states = [5,6,1,2];        % states basic motorcycle

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
% p1        ... castering mode
% p2, p3    ... weave mode
% p4        ... capsize mode

offset = 2;
p1 = p(1) - offset;
p2 = p(2) - offset;
p3 = p(3) - offset;
p4 = p(4) - offset;
poles = [p1, p2, p3, p4];

disp('Controller Matrix F:');
F = place(Arel, Brel, poles)

% check the pole location
p_real = eig(Arel-Brel*F)

