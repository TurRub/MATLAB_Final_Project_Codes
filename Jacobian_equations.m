clc
clear all

%% Calculating the Jacobian Matrix
% This code show the jacobian matrix in equation form

syms D1 t2 t3 D4 A1 A2
Des_JV = [D1 t2 t3 D4];

T01 = [1 0 0 A1; 0 1 0 0; 0 0 1 Des_JV(1); 0 0 0 1]
T12 = [cos(Des_JV(2)) -sin(Des_JV(2)) 0 A2*cos(Des_JV(2)); sin(Des_JV(2)) cos(Des_JV(2)) 0 A2*sin(Des_JV(2)); 0 0 1 0; 0 0 0 1];
T23 = [cos(Des_JV(3)) 0 sin(Des_JV(3)) 0; sin(Des_JV(3)) 0 -cos(Des_JV(3)) 0; 0 1 0 0; 0 0 0 1];
T34 = [1 0 0 0; 0 1 0 0; 0 0 1 Des_JV(4); 0 0 0 1];

% Forward Kinematics
T02 = T01 * T12
T03 = T02 * T23
T04 = T03 * T34

% Rotation Matrices
R1 = T01([1 2 3],[1 2 3]);
R2 = T12([1 2 3],[1 2 3]);
R3 = T23([1 2 3],[1 2 3]);
R4 = T34([1 2 3],[1 2 3]);

% 0Zi-1
z0 = [0;0;1];
z1 = R1 * z0;
z2 = R1*R2 * z0;
z3 = R1*R2*R3 * z0;

% On - Oi-1
Distance_P0 = T04;
Distance_P1 = T04 - T01;
Distance_P2 = T04 - T02;
Distance_P3 = T04 - T03;

P0 = Distance_P0([1 2 3],[4]);
P1 = Distance_P1([1 2 3],[4]);
P2 = Distance_P2([1 2 3],[4]);
P3 = Distance_P3([1 2 3],[4]);

% Jacobian Matrix
J1 = [z0; 0; 0; 0];
J2 = [cross(z1,P1); z1];
J3 = [cross(z2,P2); z2];
J4 = [z3; 0; 0; 0];

Jacobian = [J1 J2 J3 J4]