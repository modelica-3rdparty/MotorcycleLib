%% EigenvalueAnalysis_SL2001.m
% State Space Controller for the advanced motorcycle (SL2001)

% Contrary to the basic models in-plane mode as well as out-of-plane modes
% are taken into account in this mode. Thus, we perform an eigenvalue
% analysis with the whole system matrix

clc;
% xuyName =
%
% 1.  RearFrame.leanAngle                                ... stability
% 2.  RearFrame.leanRate                                 ... stability
% 3.  FrontFrame.Steering.phi                            ... stability
% 4.  FrontFrame.Steering.w                              ... stability
% 5.  FrontFrame.FWRevolute.phi                          ... stability
% 6.  FrontFrame.FWRevolute.w                            ... stability
% 7.  FrontFrame.TwistAxis.phi                           ... stability
% 8.  FrontFrame.TwistAxis.w                             ... stability
% 9.  RearWheel.WheelBody.PotentialFBM1.x[1]             ... not necessary
% 10. RearWheel.WheelBody.PotentialFBM1.x[3]             ... not necessary
% 11. Rider.revolute.phi                                 ... stability
% 12. Rider.revolute.w                                   ... stability
% 13. SwingingArm.RWRevolute.phi                         ... stability
% 14. SwingingArm.RWRevolute.w                           ... stability
% 15. SwingingArm.rearSuspension.actuatedPrismatic.s     ... stability
% 16. SwingingArm.rearSuspension.actuatedPrismatic.v     ... stability
% 17. stateSelect.set1.x[1]                              ... not necessary
% 18. stateSelect.set1.x[2]   (RearWheel.phi[3])         ... stability

% all but three states are necessary in order to compute the eigenvalues
% which are responsible for stability

%% load dslin.math and show the names of the states, inputs and outputs
load dslin.mat;
xuyName;

[A,B,C,D]=tloadlin('dslin.mat');

%% State variables
states = [1,2, 3,4, 5,6, 7,8, 11,12, 13,14, 15, 16, 18];

Arel = A(states, states);
Brel = B(states,:);

p = eig(Arel)

% o=ones(15,1)
% t2 = linspace(0,17,18);
% ev_new = [p0,p1,p2,p3,p4,p5,p6,p7, p8,p9,p10,p11,p12, p13, p14, p15,p16, p17];
% ev_new2 = sortrows(ev_new, -1);
% plot((o(1:14)*t2(4:18))', ev_new2(2:15,4:18)')
% axis([3 17 -30 5])
