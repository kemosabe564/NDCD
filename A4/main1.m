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


%% problem 1a
Index = "1a";
P_1a

%% problem 1b
Index = "1b";
P_1b

%% problem 1c
Index = "1c";
P_1c

%% problem 1d
Index = "1d";
P_1d

% %% problem 1e
% Index = "1e";
% P_1e


