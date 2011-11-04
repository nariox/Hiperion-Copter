% Multirotor Simulator
% Developed by PLaTooN
% Licensed under ???
% 2011

% Notes
% 1 - This simulator currently considers Thrust to be proportional to the square of the rotation
% 2 - It disregards the motor dynamics. 
% 3 - No wind or external noise

clear all;
clc;
hold off;

% Simulation parameters
N=40000; % number of samples
dT=0.001; % Simulation timestep
n=1:N;
CdT = 0.1; % Controller Timestep

% Multirotor parameters
Jphi = 1;
Jpsi = 1;
Jtheta = 1;
Mass = 0.1;
L = 50;
l = L/sqrt(2);
CT = 0.1; % Constante aerodinamica transversal
CN = 1; % Constante aerodinamica normal

% Input
PhiInput=zeros(N,1);
PsiInput=zeros(N,1);
ThetaInput=zeros(N,1);
ThrottleInput=zeros(N,1);

% Control parameters
PIDphi.P=6;
PIDphi.D=1;
PIDpsi.P=6;
PIDpsi.D=1;
PIDtheta.P=6;
PIDtheta.D=1;

% Physical Parameters
G=9.8;