%%%%%
%%% SC42100 Networked and Distributed Control System
%%%%%
clc
close all
clear all

%% add path
addpath('./src')

%% setup and initialization
W = [0.75, 0.25, 0,    0;
     0.25, 0.5,  0.25, 0;
     0,    0.25, 0.5,  0.25;
     0,    0,    0.25, 0.75];

init

P_2
problem = "1";
Index = "2_1a";

if problem == "1"
    P_2a
else
    P_2b
end
% %% problem 2a
% Index = "2_1a";
% P_2a
% 
% %% problem 2b
% Index = "2_1a";
% P_2b