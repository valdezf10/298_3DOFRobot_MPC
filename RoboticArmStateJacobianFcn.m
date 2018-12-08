%%%%%%%%%%%%% NEED TO WORK ON THIS FNC %%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%

function [A, B] = RoboticArmStateJacobianFcn(x, u)
% Jacobian of model equations for the free-flying robot example.
%
% States:
%   x(1:3)  Theta
%   x(4:6)  Thetadot
%
% Inputs:
%   u(1:3) tau, input torque

% Parameters
%% parameters
% From KUKA r360 2835
L1 = 1; 
L2 = 1; 
L3 = 1; 
m1 = 1; 
m2 = 1; 
m3 = 1; 
R1 = 1; 

g = 9.81; % Gravity

% Variables
theta = x(3);
T1 = u(1) - u(2);
T2 = u(3) - u(4);

% Linearize the state equations at the current condition
A = zeros(6,6);
A(1,4) = 1;
A(2,5) = 1;
A(3,6) = 1;
A(4,3) = -(T1 + T2)*sin(theta);
A(5,3) =  (T1 + T2)*cos(theta);
B = zeros(6,4);
B(4,:) = cos(theta)*[1 -1 1 -1];
B(5,:) = sin(theta)*[1 -1 1 -1];
B(6,:) = [alpha*[1 -1], -beta*[1 -1]];
