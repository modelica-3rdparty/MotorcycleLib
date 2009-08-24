%% StateSpaceController_SL2001.m
% State Space Controller for the advanced motorcycle (SL2001)

% Contrary to the basic models in-plane mode as well as out-of-plane modes
% are taken into account in this mode. Thus, we perform an eigenvalue
% analysis with the whole system matrix

clc;

%% load dslin.math and show the names of the states, inputs and outputs
load dslin.mat;
xuyName;

[A,B,C,D]=tloadlin('dslin.mat');

%% State variables
%states = [1,2,3,4, 7,8, 11,12, 13,14, 15, 16, 17, 18];
states = [1,2, 3,4, 5,6, 7,8, 11,12, 13,14, 15, 16, 18];
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
% compute the eigenvalues of Arel

disp(' ')
disp('Compute the poles (p) of the vehicle:')
p = eig(Arel)

% pole placement according to Arel
offset = 5;
poles = p;
% solely shift the weave mode eigenvalues towards the left-half plane
poles(11) = p(11) - offset;
poles(12) = p(12) - offset;

disp('Controller Matrix F:');
F = place(Arel, Brel, poles);

F_con = F(1:4)

% check the pole location
p_real = eig(Arel-Brel*F);

