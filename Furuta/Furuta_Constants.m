clear;
clc;

g = 9.81;
% 1 - Arm
% 2 - Pendulum 

m1 = 0.380;
m2 = 0.054;
L1 = 0.066; 
L2 = 0.146;  
M = 0.044;

%omega_n = 8.4908;
%zeta = 0.0145;
%kb = 2*zeta*omega_n*J;    %Pendulum friction - TauTheta
%J = m2*g*L2/(omega_n^2);  %Pedulum axis

J = 3.5256e-04;            %System
kb_p = 4.7940e-04 ;
kb_m = 6.75e-4 ;
b = 2.5482e-4;

%ke = 0.182;
%Re = 3.6;
ke = 0.5; % kt Motor 2e-2
Re = 14.5;  %Medido

alpha = J + (M+m1/3+m2)*L1^2;
beta = (M + m2/3)*L2^2;
gamma = (M + m2/2)*L2*L1;
sigma = (M + m2/2)*g*L2;

%% Simulation parameters

initial_state = pi;
Ts = 0.001;
dtDisc = 0.01;
Reference = [0 0 0 0];
Zn = 3;                 %Dead Zone
GSwingUp = 2.4738; %3          %Swing up gain

StepX = 10;
distrub = 12;
disturb = distrub*pi/180;

%% Linearization

% A matrix
A = zeros(4,4);
A(1,2) = 1;
A(2,3) = -(sigma*gamma)/(alpha*beta-gamma^2);
A(3,4) = 1;
A(4,3) = (alpha*sigma)/(alpha*beta-gamma^2);

% B matrix
B = zeros(4,2);
B(2,1) = beta/(alpha*beta-gamma^2);
B(2,2) = -gamma/(alpha*beta-gamma^2);
B(4,1) = -gamma/(alpha*beta-gamma^2);
B(4,2) = alpha/(alpha*beta-gamma^2);

% C matrix
C = [0 0 1 0;
     0 0 0 1];

%% Pseudo Linear System

% Pseudo A matrix
Ap = zeros(4,4);

Ap(1,2) = 1;

Ap(2,1) = 0;      Ap(2,2) = -B(2,1)*(ke^2/Re + kb_m); 
Ap(2,3) = A(2,3); Ap(2,4) = -B(2,2)*kb_p;

Ap(3,4) = 1;

Ap(4,1) = 0;      Ap(4,2) = -B(4,1)*(ke^2/Re + kb_m); 
Ap(4,3) = A(4,3); Ap(4,4) = -B(4,2)*kb_p;

% Pseudo B matrix

Bp = zeros(4,1);

Bp(2) = B(2,1)*ke/Re;  
Bp(4) = B(4,1)*ke/Re;   

% Controlability and Observability
Control = rank(ctrb(Ap,Bp));
Observ = rank(obsv(Ap,C));

% State Feedback Control

%K = place(Ap,Bp,[-5 -4 -2+2j -2-2j]);  
%K = place(Ap,Bp,[-8+8j -8-8j  -4+4j -4-4j]);


Q = [2 0 0 0;
     0 2 0 0;
     0 0 10 0;
     0 0 0 1];
R = 10;
[K, ~, E] = lqr(Ap,Bp,Q,R);

%% Down position (pi)

% Pseudo A matrix around pi
Ap2 = zeros(4,4);

Ap2(1,2) = 1;

Ap2(2,1) = 0;      Ap2(2,2) = -B(2,1)*(ke^2/Re + kb_m); 
Ap2(2,3) = A(2,3); Ap2(2,4) = B(2,2)*kb_p;

Ap2(3,4) = 1;

Ap2(4,1) = 0;      Ap2(4,2) = B(4,1)*(ke^2/Re + kb_m); 
Ap2(4,3) = -A(4,3); Ap2(4,4) = -B(4,2)*kb_p;

% Pseudo B matrix around pi

Bp2 = zeros(4,1);

Bp2(2) = B(2,1)*ke/Re;  
Bp2(4) = -B(4,1)*ke/Re;  

K2 = place(Ap2,Bp2,[-5 -4 -2+2j -2-2j]); 
