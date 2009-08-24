% ButterFil.m
% Butterworth filtered path generation
% A path is generated w.r.t. random numbers
% These random numbers are filtered in such a way that the result is a
% smooth trackable path

%clear all;
clc;

%n = 91;
n = 200;
t=linspace(0,50,n);
v = 6;
x = v*t;

y = 80*rand(size(x));
% Offset elimination
y = y - mean(y);
% Butterworth Filter
%[b,a] = butter(5,0.12);
[b,a] = butter(7,0.05);

f = filter(b,a,y);
figure(1)
plot(x, y, x, f)
title('Path (Roadway)');
xlabel('fixed x-direction [m]');
ylabel('lateral distance [m]');
legend('original data', 'filtered data')

%% Calculte the real path length
% until now the traveled path of the motorcycle is calculated via
% x = v*t
% Thus the function just valid for very small lateral deviations (y-values)
path(1) = 0;
for i = 2:length(x)
    l(i) = sqrt( (x(i)-x(i-1))^2 + (f(i)-f(i-1))^2 );
    path(i) = path(i-1) + l(i);
end

%% Curvature calcutlation
% symbolic precalculation

delta = x(2)-x(1);
en = diff(f, 2)/delta^2;
den = sqrt(1 + (diff(f)/delta).^2 ).^3;

C = en./den(1:n-2);
figure(2)
plot(x(1:n-2), C, 'o')

%% motorcycle lean angle
g = 9.81;
phi = atan(v^2*C/g);
figure(3)
plot(x(1:n-2), phi)

%% Path angle calculation
psi = diff(f)/delta;
figure(4)
plot(x(1:n-1), psi)

%% store path and curvature in table functions
% Positon Table
pos_tab = [path(1:n-2)', f(1:n-2)'];
save('Position.mat', 'pos_tab', '-v4');

% Curvature Table
cur_tab = [path(1:n-2)', C(1:n-2)'];
save('Curvature.mat', 'cur_tab', '-v4');

% Lean Angle
lean_tab = [path(1:n-2)', phi(1:n-2)'];
save('LeanAngle.mat', 'lean_tab', '-v4');

% yaw angle
yaw_tab = [path(1:n-2)', psi(1:n-2)'];
save('YawAngle.mat', 'yaw_tab', '-v4');
