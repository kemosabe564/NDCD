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

NCS.h = h;

a1 = 1.0;
b1 = 1.0;
NCS.poles = [-a1+b1*1i -a1-b1*1i];

NCS.K = place(NCS.A, NCS.B, NCS.poles);
eig(NCS.A - NCS.B * NCS.K);

clear ans a1 b1

%% Q4.1
% small delay with tau = 0.5h
small_tau = 0.5*h;

NCS_small_delay.Fx = expm(NCS.A * NCS.h);

temp = expm(NCS.A * s);

NCS_small_delay.Fu = int(temp, s, h-small_tau, h) * NCS.B;
NCS_small_delay.G1 = int(temp, s, 0, h-small_tau) * NCS.B;

clear temp
% augmentation
NCS_small_delay.F = [NCS_small_delay.Fx NCS_small_delay.Fu zeros(2, 1) zeros(2, 1);
                     zeros(1, 2) 0 0 0;
                     zeros(1, 2) 1 0 0;
                     zeros(1, 2) 0 1 0];
NCS_small_delay.G = [NCS_small_delay.G1; 1; 0; 0];
% NCS_small_delay.F
% NCS_small_delay.G

% mid delay with tau = h
mid_tau = h;

NCS_mid_delay.Fx = expm(NCS.A * NCS.h);

temp = expm(NCS.A * s);

NCS_mid_delay.Fu1 = int(temp, s, h-mid_tau, h) * NCS.B;
NCS_mid_delay.Fu2 = int(temp, s, 0, h-mid_tau) * NCS.B;

clear temp
% augmentation
NCS_mid_delay.F = [NCS_mid_delay.Fx NCS_mid_delay.Fu1 NCS_mid_delay.Fu2 zeros(2, 1);
                   0, 0, 0, 0, 0;
                   0, 0, 1, 0, 0;
                   0, 0, 0, 1, 0];

NCS_mid_delay.G = [0; 0; 1; 0; 0];


% large delay with tau = h
large_tau = 2*h;

NCS_large_delay.Fx = expm(NCS.A * NCS.h);

temp = expm(NCS.A * s);

NCS_large_delay.Fu2 = int(temp, s, large_tau-2*NCS.h, large_tau) * NCS.B;
NCS_large_delay.Fu3 = int(temp, s, 0, large_tau-2*NCS.h) * NCS.B;

clear temp
% augmentation
NCS_large_delay.F = [NCS_large_delay.Fx, zeros(2,1), NCS_large_delay.Fu2, NCS_large_delay.Fu3;
                   0, 0, 0, 0, 0;
                   0, 0, 1, 0, 0;
                   0, 0, 0, 1, 0];
NCS_large_delay.G = [0; 0; 1; 0; 0];

% NCS_large_delay.F
% NCS_large_delay.G

NCS_small_delay.A = NCS_small_delay.F - NCS_small_delay.G * [NCS.K 0 0 0];

NCS_mid_delay.A = NCS_mid_delay.F - NCS_mid_delay.G * [NCS.K 0 0 0];

NCS_large_delay.A = NCS_large_delay.F - NCS_large_delay.G * [NCS.K 0 0 0];

%% Q4.2
% searching phase
h_samples = 0.2 : 0.005 : 0.3;

n_size = 5;

iter = cell(length(h_samples), 2);

h_range = zeros(1,2);

for i = 1:length(h_samples)
    
    F1 = double(subs(NCS_small_delay.F, h, h_samples(i)));
    G1 = double(subs(NCS_small_delay.G, h, h_samples(i)));
    
    F2 = double(subs(NCS_mid_delay.F, h, h_samples(i)));
    G2 = double(subs(NCS_mid_delay.G, h, h_samples(i)));
    
    F3 = double(subs(NCS_large_delay.F, h, h_samples(i)));
    G3 = double(subs(NCS_large_delay.G, h, h_samples(i)));
    
    
    cvx_clear
    cvx_begin sdp
        variable X(n_size, n_size) semidefinite
        variable Y(1, n_size)

        subject to
            [X, F1*X - G1*Y;
            (F1*X - G1*Y)', X] >= 0.001 * eye(2*n_size);
            
            [X, F2*X - G2*Y;
            (F2*X - G2*Y)', X] >= 0.001 * eye(2*n_size);
            
            [X, F3*X - G3*Y;
            (F3*X - G3*Y)', X] >= 0.001 * eye(2*n_size);
            
    cvx_end
    
    if(cvx_status == "Solved")
        iter{i, 2} = 1;
%         iter{i, 3} = X;
%         iter{i, 4} = inv(X);
        K = Y/X;
        iter{i, 3} = K;
        
        h_test = 0;
        hmax = 0; hmin = 0; step = 0.005;
        
        while true
            h_test = h_test + step;
            
            F1 = double(subs(NCS_small_delay.F, h, h_test));
            G1 = double(subs(NCS_small_delay.G, h, h_test));
            
            F2 = double(subs(NCS_mid_delay.F, h, h_test));
            G2 = double(subs(NCS_mid_delay.G, h, h_test));
            
            F3 = double(subs(NCS_large_delay.F, h, h_test));
            G3 = double(subs(NCS_large_delay.G, h, h_test));
            
            cvx_clear
            cvx_begin sdp
    
                variable P(n_size, n_size) semidefinite
    
                (F1 - G1*K)' * P * (F1 - G1*K) - P <= -0.001 * eye(n_size);
                (F2 - G2*K)' * P * (F2 - G2*K) - P <= -0.001 * eye(n_size);
                (F3 - G3*K)' * P * (F3 - G3*K) - P <= -0.001 * eye(n_size);
    
            cvx_end
                
            if(cvx_status == "Solved" && hmin == 0)
                hmin = h_test;
                
            elseif(hmin ~= 0 && cvx_status == "Infeasible")
                hmax = h_test - step;
                break;
            end
        end        
        
        iter{i, 4} = hmin;
        iter{i, 5} = hmax;
        if((h_range(2)-h_range(1)) < (hmax-hmin))
            index = i;
        end
        
    else    
        iter{i, 2} = 0;
        
    end
    iter{i, 1} = h_samples(i);
    
end

%% Q4.3





%% Q4.4

% searching phase
h_samples = 0.2 : 0.05 : 1.0;

n_size = 5;

iter = cell(length(h_samples), 2);

h_range = zeros(1,2);

for i = 1:length(h_samples)
    
    F1 = double(subs(NCS_small_delay.F, h, h_samples(i)));
    G1 = double(subs(NCS_small_delay.G, h, h_samples(i)));
    
    F2 = double(subs(NCS_mid_delay.F, h, h_samples(i)));
    G2 = double(subs(NCS_mid_delay.G, h, h_samples(i)));
    
    F3 = double(subs(NCS_large_delay.F, h, h_samples(i)));
    G3 = double(subs(NCS_large_delay.G, h, h_samples(i)));
    
    
    cvx_clear
    cvx_begin sdp
        variable X(n_size, n_size) semidefinite
        variable Y1(1, n_size)
%         variable Y2(1, n_size)
%         variable Y3(1, n_size)

        subject to
            [X, F1*X - G1*Y1;
            (F1*X - G1*Y1)', X] >= 0.001 * eye(2*n_size);
            
            [X, F2*X - G2*Y1;
            (F2*X - G2*Y1)', X] >= 0.001 * eye(2*n_size);
            
            [X, F3*X - G3*Y1;
            (F3*X - G3*Y1)', X] >= 0.001 * eye(2*n_size);
            
    cvx_end
    
    if(cvx_status == "Solved")
        iter{i, 2} = 1;
%         iter{i, 3} = X;
%         iter{i, 4} = inv(X);
        K = Y1/X;
        
        
        iter{i, 3} = K;
        
        h_test = 0;
        hmax = 0; hmin = 0; step = 0.05;
        
        while true
            h_test = h_test + step;
            
            F1 = double(subs(NCS_small_delay.F, h, h_test));
            G1 = double(subs(NCS_small_delay.G, h, h_test));
            
            F2 = double(subs(NCS_mid_delay.F, h, h_test));
            G2 = double(subs(NCS_mid_delay.G, h, h_test));
            
            F3 = double(subs(NCS_large_delay.F, h, h_test));
            G3 = double(subs(NCS_large_delay.G, h, h_test));
            
            cvx_clear
            cvx_begin sdp
    
                variable P(n_size, n_size) semidefinite
    
%                 (F1 - G1*K)' * P * (F1 - G1*K) - P <= -0.001 * eye(n_size);
%                 (F2 - G2*K)' * P * (F2 - G2*K) - P <= -0.001 * eye(n_size);
%                 (F3 - G3*K)' * P * (F3 - G3*K) - P <= -0.001 * eye(n_size);
                ((F1-G1*K)*(F2-G2*K)*(F3-G3*K))'*P*((F1-G1*K)*(F2-G2*K)*(F3-G3*K))-P <= -0.01*eye(n_size);
    
            cvx_end
                
            if(cvx_status == "Solved" && hmin == 0)
                hmin = h_test;
                
            elseif(hmin ~= 0 && cvx_status == "Infeasible")
                hmax = h_test - step;
                break;
            end
        end        
        
        iter{i, 4} = hmin;
        iter{i, 5} = hmax;
        if((h_range(2)-h_range(1)) < (hmax-hmin))
            index = i;
        end
        
    else    
        iter{i, 2} = 0;
        
    end
    iter{i, 1} = h_samples(i);
    
end




