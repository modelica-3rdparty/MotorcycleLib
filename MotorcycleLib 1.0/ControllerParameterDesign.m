%% ControllerParameterDesign.m
% W.r.t. this program the velocity dependent controller parameters are
% determined

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
Crel = eye(2);
Drel = 0;

vehicle = ss(Arel, Brel, Crel, Drel);

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
p = pole(vehicle)

offset = 3;
p1 = - 5;
p2 = - 3;

disp('Controller Matrix F:');
F = place(Arel, Brel, [p1, p2])

% check the pole location
p_real = eig(Arel-Brel*F)

%% Prefilter Design
%V = inv([Crel * inv(Brel*F-Arel) * Brel])

%% Stability of the closed-loop system

% without reference input (steady state --> x = 0)
Ac = Arel - Brel*F;
Bc = Brel;
Cc = Crel;
Dc = 0;

Sys = ss(Ac, Bc, Cc, Dc);

% Initial conditions
t = 0:0.001:10;
x0 = [5*pi/180; 0];
[phi,t,X] = initial(Sys, x0, t);
%setPoint = ss(Ac, Bc, -F, 0)
%initial(setPoint, x0)
plot(t, phi)
legend('Lean Angle','Lean Rate')
grid on;

% with reference input (steady state --> x != 0)
v = 6;
% motorcycle parameter
p = 1.414;                                       % wheelbase
eps = 27*pi/180;                                 % head angle
% reference input calculation
phi = 15*pi/180;                                 % lean angle
R = v^2 / (9.81*tan(phi));                       % curve radius
delta = atan( p*cos(phi) / (R*cos(eps)) );   % lean angel
% reference set points xs
xs = [delta, 0, phi, 0]

As = Arel - Brel*F;
Bs = Brel*F*xs;
Cs = Crel;
Ds = 0;

Sys2 = ss(As, Bs, Cs, Ds);
[phi,t,X] = initial(Sys, x0, t);
plot(t, phi)
legend('Lean Angle','Lean Rate')
grid on;
