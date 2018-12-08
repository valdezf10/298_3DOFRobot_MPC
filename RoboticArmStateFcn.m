function dxdt = RoboticArmStateFcn(x, u)
%% Continuous-time nonlinear dynamic model of a pendulum on a cart 
%
% States:
%   x(1:3)  Theta
%   x(4:6)  Thetadot
%
% Inputs:
%   u(1:3) tau, input torque

%#codegen

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
%% Obtain x, u and y
% transpose to make column vecotors
% x
theta = x(1:3)';
thetadot = x(4:6)';
% u
Tau = u';
% y
y = x';

%% Compute dxdt

%Mass Matrix Parameter Construction
M_11 = .5*m1*(R1)^2 + (1/3)*m2*(L2)^2*(cos(theta(2)))^2 +(1/3)*m3*(L3)^2*cos(theta(2)+theta(3)) + m3*(L2)^2*(cos(theta(2)))^2 + m3*L2*L3*cos(theta(2)+theta(3))*cos(theta(2));
M_12 = 0; 
M_13 = 0;
M_21 = 0;
M_22 = (1/3)*m2*(L2)^2 + (1/3)*m3*(L3)^2 + m3*(L2)^2 + m3*L2*L3*cos(theta(3));
M_23 = (1/3)*m3*(L3)^2 + m3*(L2)^2 + (1/3)*m3*L2*L3*cos(theta(3));
M_31 = 0; 
M_32 = (1/3)*m3*(L3)^2 + m3*(L2)^2 + (1/3)*m3*L2*L3*cos(theta(3));
M_33 = (1/3)*m3*(L3)^2; 

% Coriolis Matrix Parameter Construction: 
C_11 = (-(4/3)*m2*(L2)^2*sin(2*theta(2))-(1/3)*m3*(L3)^2*sin(2*(theta(2)+theta(3)))-m3*L2*L3*sin(2*theta(2) + theta(3)))*thetadot(2)*thetadot(1) +(-(1/3)*m3*(L2)^2*sin(2*(theta(2)+theta(3))-m3*L2*L3*cos(theta(2))*sin(theta(2)+theta(3))))*thetadot(3)*thetadot(1) ; 
C_21 = (-m3*L2*L3*sin(theta(3)))*thetadot(2)*thetadot(3) + (-(1/2)*m3*L2*L3*sin(theta(3)))*(thetadot(3))^2 + ((1/6)*m2*(L2)^2*sin(2*theta(2)) + (1/6)*m3*(L3)^2*sin(2*(theta(2)+theta(3))) + (1/2)*m3*(L2)^2*sin(2*theta(2)) + (1/2)*m3*L2*L3*sin(2*theta(2)+theta(3)))*(thetadot(1))^2; 
C_31 = ((1/2)*m3*L2*L3*sin(theta(3)))*(thetadot(2))^2 + ((1/6)*m3*(L3)^2*sin(2*(theta(2)+theta(3))) + (1/2)*m3*L2*L3*cos(theta(2))*sin(theta(2)+theta(3)))*(thetadot(1))^2; 

% Gravity Matrix Parameter Construction:
G_11 = 0; 
G_21 = .5*m2*g*L2*cos(theta(2)) + .5*m3*g*L3*cos(theta(2)+theta(3)) + m3*g*L2*cos(theta(2)); 
G_31 = .5*m3*g*L3*cos(theta(2)+theta(3)); 

%Mass Matrix: 
M = [M_11, M_12, M_13; 
     M_21, M_22, M_23 ; 
     M_31, M_32, M_33];
%Coriolis Matrix: 
C = [C_11;C_21;C_31];
%Gravity Matrix: 
G = [G_11;G_21;G_31]; 

%angular acceleration 
thetadotdot = inv(M)*(Tau'-C-G); 

%initialize
dxdt = zeros(6,1);

%thetadot
dxdt(1:3) = x(4:6);

%thetadotdot
dxdt(4:6) = thetadotdot;