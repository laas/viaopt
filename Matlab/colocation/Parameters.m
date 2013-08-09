clc; clear all;

% Initialization
T = 0;
StateInit = [0 0 0 0];
StateObj = 0.3;
% alpha0 = ones(n,1)*2*(StateObj-StateInit(3))/(n*dT)^2;

% Parameters of the optimization
TimeSliding = 1;
NbWindows = 1000;
n = 10; 
m = 417;
Duration = 0.01;

alpha0 = ones(n,1)*2*(StateObj-StateInit(3));




% Start Optimization
master(T,StateInit,StateObj,alpha0,TimeSliding,NbWindows,n,m,Duration);