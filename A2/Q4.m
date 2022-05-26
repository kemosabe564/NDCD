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

NCS.K = [-2 1.6];
% NCS.K = [-2 0];

NCS.h = 0.2;

%% 4.1

syms h tau s

NCS_small_delay.Fx = expm(NCS.A*h);

temp = expm(NCS.A*s);
NCS_small_delay.Fu = int(temp, s, h-tau, h)*NCS.B;

NCS_small_delay.G1 = int(temp, s, 0, h-tau)*NCS.B;

NCS_small_delay.F = [NCS_small_delay.Fx, NCS_small_delay.Fu;
                     0 0 0];
NCS_small_delay.G = [NCS_small_delay.G1;
                    1];

[V, J] = jordan(sym(10*NCS.A));

NCS_small_delay.Q = inv(V);
NCS_small_delay.J = J/10;

clear V J temp

NCS_small_delay.F
% 
% NCS_small_delay.G

%% 4.2
NCS_polytopic.alpha_1 = expm(-3*(h-tau)); 
NCS_polytopic.alpha_2 = expm((h-tau)); 


NCS_polytopic.F1 = [0 0 5/24; 0 0 0; 0 0 0];
NCS_polytopic.F2 = [0 0 5/8; 0 0 -1; 0 0 0];
NCS_polytopic.F0 = NCS_small_delay.F - NCS_polytopic.F1*NCS_polytopic.alpha_1 - NCS_polytopic.F2*NCS_polytopic.alpha_2;

NCS_polytopic.G0 = [5/6; -1; 1];
NCS_polytopic.G1 = [-5/24;0; 0];
NCS_polytopic.G2 = [-5/8; 1; 0];


K = [NCS.K 0];


h_samples = 0:0.01:0.5;

result = zeros(1, length(h_samples));

for i=1:1:length(h_samples)


    h_sample = h_samples(i);

    tau_lb = 0; tau_ub = h_sample;

    alpha_1_lb = double(subs(NCS_polytopic.alpha_1, [h, tau], [h_sample, tau_lb]));
    alpha_1_ub = double(subs(NCS_polytopic.alpha_1, [h, tau], [h_sample, tau_ub]));
    alpha_2_lb = double(subs(NCS_polytopic.alpha_2, [h, tau], [h_sample, tau_lb]));
    alpha_2_ub = double(subs(NCS_polytopic.alpha_2, [h, tau], [h_sample, tau_ub]));
 

    F0 = subs(NCS_polytopic.F0, [h, tau], [h_sample, tau_ub])
    G0 = subs(NCS_polytopic.G0, [h, tau], [h_sample, tau_ub])
    F1_vertices = double(F0 + alpha_1_lb * NCS_polytopic.F1 + alpha_2_lb * NCS_polytopic.F2);
    G1_vertices = double(G0 + alpha_1_lb * NCS_polytopic.G1 + alpha_2_lb * NCS_polytopic.G2);
    F2_vertices = double(F0 + alpha_1_lb * NCS_polytopic.F1 + alpha_2_ub * NCS_polytopic.F2);
    G2_vertices = double(G0 + alpha_1_lb * NCS_polytopic.G1 + alpha_2_ub * NCS_polytopic.G2);
    F3_vertices = double(F0 + alpha_1_ub * NCS_polytopic.F1 + alpha_2_lb * NCS_polytopic.F2);
    G3_vertices = double(G0 + alpha_1_ub * NCS_polytopic.G1 + alpha_2_lb * NCS_polytopic.G2);
    F4_vertices = double(F0 + alpha_1_ub * NCS_polytopic.F1 + alpha_2_ub * NCS_polytopic.F2);
    G4_vertices = double(G0 + alpha_1_ub * NCS_polytopic.G1 + alpha_2_ub * NCS_polytopic.G2);


    cvx_clear
    cvx_begin sdp

        variable P(3,3) semidefinite

        subject to
            (F1_vertices - G1_vertices*K)'*P*(F1_vertices - G1_vertices*K) - P <= -0.001*P;
            (F2_vertices - G2_vertices*K)'*P*(F2_vertices - G2_vertices*K) - P <= -0.001*P;
            (F3_vertices - G3_vertices*K)'*P*(F3_vertices - G3_vertices*K) - P <= -0.001*P;
            (F4_vertices - G4_vertices*K)'*P*(F4_vertices - G4_vertices*K) - P <= -0.001*P;
            P >= 0.001*eye(3);
    cvx_end

    if (cvx_status=="Solved")    
        result(i)=1;
    end


end


figure

% contour(result)
result(1)=1;


scatter(h_samples,result)

xlabel("sampling time h", 'FontSize', 16); 
ylabel("is feasible?", 'FontSize', 16);

title("Feasible Check with 4 Vertices", 'FontSize', 16)


%% 4.3
h_samples = 0:0.025:1;
tau_samples = h_samples;
plot_h = zeros([1, (length(h_samples)*length(tau_samples))]);
plot_tau = zeros([1, (length(h_samples)*length(tau_samples))]);
results = zeros([1, (length(h_samples)*length(tau_samples))]);

for i = 1:length(h_samples)
    tau_samples = 0 : h_samples(i)/40 : h_samples(i)
    for j = 1:length(tau_samples)
        
        if (tau_samples(j)>0.01)
            tau_lb = tau_samples(j)-0.01;
        else
            tau_lb = tau_samples(j);
        end
        tau_ub = tau_samples(j)+0.01;

        alpha_1_lb = double(subs(NCS_polytopic.alpha_1, [h, tau], [h_samples(i), tau_lb]));
        alpha_1_ub = double(subs(NCS_polytopic.alpha_1, [h, tau], [h_samples(i), tau_ub]));
        alpha_2_lb = double(subs(NCS_polytopic.alpha_2, [h, tau], [h_samples(i), tau_lb]));
        alpha_2_ub = double(subs(NCS_polytopic.alpha_2, [h, tau], [h_samples(i), tau_ub]));
      
        F0 = subs(NCS_polytopic.F0, [h, tau], [h_samples(i), tau_ub]);
        G0 = subs(NCS_polytopic.G0, [h, tau], [h_samples(i), tau_ub]);
        F1_vertices = double(F0 + alpha_1_lb * NCS_polytopic.F1 + alpha_2_lb * NCS_polytopic.F2);
        G1_vertices = double(G0 + alpha_1_lb * NCS_polytopic.G1 + alpha_2_lb * NCS_polytopic.G2);
        F2_vertices = double(F0 + alpha_1_lb * NCS_polytopic.F1 + alpha_2_ub * NCS_polytopic.F2);
        G2_vertices = double(G0 + alpha_1_lb * NCS_polytopic.G1 + alpha_2_ub * NCS_polytopic.G2);
        F3_vertices = double(F0 + alpha_1_ub * NCS_polytopic.F1 + alpha_2_lb * NCS_polytopic.F2);
        G3_vertices = double(G0 + alpha_1_ub * NCS_polytopic.G1 + alpha_2_lb * NCS_polytopic.G2);
        F4_vertices = double(F0 + alpha_1_ub * NCS_polytopic.F1 + alpha_2_ub * NCS_polytopic.F2);
        G4_vertices = double(G0 + alpha_1_ub * NCS_polytopic.G1 + alpha_2_ub * NCS_polytopic.G2);

        cvx_clear
        cvx_begin sdp

            variable P(3,3) semidefinite

            subject to

                (F1_vertices - G1_vertices*K)'*P*(F1_vertices - G1_vertices*K) - P <= -0.001*P;
                (F2_vertices - G2_vertices*K)'*P*(F2_vertices - G2_vertices*K) - P <= -0.001*P;
                (F3_vertices - G3_vertices*K)'*P*(F3_vertices - G3_vertices*K) - P <= -0.001*P;
                (F4_vertices - G4_vertices*K)'*P*(F4_vertices - G4_vertices*K) - P <= -0.001*P;
                P >= 0.001*eye(3);
                P-P' == 0;
        cvx_end

        k = ((i-1)*length(tau_samples)+j);
        
        plot_h(1, k) = h_samples(i);
        plot_tau(1, k) = tau_samples(j);
        
        if (cvx_status=="Solved")
        
            results(1, k) = 2;
        
        end
        
    end
end


figure
inx = find(results >= 1);
plot3(plot_h(inx), plot_tau(inx), results(inx), 'o', 'color', 'b', 'MarkerSize', 4)
grid on
hold on
inx = find(results < 1);
plot3(plot_h(inx), plot_tau(inx), results(inx), 'o', 'color', 'r', 'MarkerSize', 4)
view(90,90)
xlabel("sampling time h", 'FontSize', 16); 
ylabel("time delay tau", 'FontSize', 16);
zlabel("\rho(F_{cl}(h, \tau))", 'FontSize', 16);
title("Feasible Area", 'FontSize', 16)




