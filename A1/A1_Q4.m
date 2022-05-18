%%
% 5281873
clc
close all
clear all

% setup
a = 5; b = 8; c = 3;

NCS.A = [a-b 0.5-c;
         0   1];

NCS.B = [0; 1];

NCS.nx = 2;

syms h s

i = 1.0;
q = 1.0;
NCS.poles = [-i+q*i -i-q*i];

NCS.K = place(NCS.A, NCS.B, NCS.poles)
eig(NCS.A - NCS.B * NCS.K)

clear ans i q

%% Q4.1

% small delay

small_tau = 0.5*h;

