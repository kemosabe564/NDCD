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

%% 3.1 

syms hl s

F = expm(NCS.A*NCS.h) - int(expm(NCS.A*s),s,0,NCS.h)*NCS.B*NCS.K
NCS_Determinism_loss.F_interval = vpa(F);

NCS_Determinism_loss.F_total_set = [];

for i=1:1:5
    F_extra = expm(NCS.A * (i*NCS.h - NCS.h));
    F_total = F_extra * NCS_Determinism_loss.F_interval;
    NCS_Determinism_loss.F_total_set = [NCS_Determinism_loss.F_total_set; F_total];
end

NCS_Determinism_loss.F_total_set = double(NCS_Determinism_loss.F_total_set)


for i_upperbound = 1:1:5

    cvx_clear

    cvx_begin sdp

    variable P(2,2) semidefinite

    subject to
            for i = 1:1:i_upperbound
                NCS_Determinism_loss.F_total_set(2*i-1:2*i,:)' * P * NCS_Determinism_loss.F_total_set(2*i-1:2*i,:) - P <= -0.001*eye(NCS.nx);
            end
    cvx_end

    if (cvx_status ~= "Solved")    
        fprintf("maximum delta is:");
        disp(i_upperbound-1)
        break;
    end

end

clear F_total F_extra F i i_upperbound

%% 3.2
NCS_Bernoulli_loss.A0 = double(vpa(expm(NCS.A*NCS.h) - int(expm(NCS.A*s), s, 0, NCS.h)*NCS.B*NCS.K, 4));
NCS_Bernoulli_loss.A1 = double(expm(NCS.A*NCS.h))

p_ub = 1; p_lb=0;
delta = 1; 
iterations = [];

while (delta > 0.0001)

    p_sample = (p_ub+p_lb)/2;
    p = p_sample;

    % LMIs
    cvx_clear
    cvx_begin sdp

        variable P(2,2) semidefinite

        subject to
            P - (1-p)*NCS_Bernoulli_loss.A0' * P * NCS_Bernoulli_loss.A0 - p*NCS_Bernoulli_loss.A1' * P * NCS_Bernoulli_loss.A1 >= 0.0001*eye(NCS.nx);
    
    cvx_end

    if(cvx_status == "Solved")
        temp_iterations=[p_sample,"Solved"];
        iterations=[iterations;temp_iterations];
        p_lb = p_sample;
    else
        temp_iterations=[p_sample,"Infeasible"];
        iterations=[iterations;temp_iterations];
        p_ub = p_sample;
    end
    
    delta = p_ub-p_lb;
end
NCS_Bernoulli_loss.p_max = p_ub;
fprintf("maximum p is:");
disp(NCS_Bernoulli_loss.p_max)

clear delta p_lb p_ub p_sample p  



%% 3.3
p_test = 0.58;
p_test = 0.8;
Total_Timestep = 100;
packet_loss_history = zeros(1, Total_Timestep);

x_0 = [100; 100];

x = zeros(NCS.nx,Total_Timestep);

for iTime = 1:1:Total_Timestep
    q = rand(1);
    if (q < p_test)
        % packet loss happen
        packet_loss_history(iTime) = 1;
        if (iTime == 1)
            x(:, iTime) = NCS_Bernoulli_loss.A1*x_0;
        else
            x(:, iTime) = NCS_Bernoulli_loss.A1*x(:,iTime-1);
        end
    else
        % packet have not lost
        packet_loss_history(iTime) = 0;
        if (iTime == 1)
            x(:,iTime) = NCS_Bernoulli_loss.A0*x_0;
        else
            x(:,iTime) = NCS_Bernoulli_loss.A0*x(:,iTime-1);
        end
    end
end


plot(1:1:Total_Timestep, x)
xlabel("time", 'FontSize', 16); 
ylabel("state value", 'FontSize', 16);
title("Evolution of state value(p = 0.9)", 'FontSize', 16)

clear Total_Timestep packet_loss_history x x_0 q iTime p_test
 

%% 3.4
lambda_ub = 1; lambda_lb = 0;

delta = 1;
iterations = [];
L_ub = 100;
while (delta > 0.001)

    lambda_sample = (lambda_ub+lambda_lb)/2;

    lambda = lambda_sample;

    % LMIs
    cvx_clear
    cvx_begin sdp
        variable P(2,2) semidefinite
        subject to
            NCS_Bernoulli_loss.A0' * P * NCS_Bernoulli_loss.A0 - lambda*P <= -0.01*eye(NCS.nx)
            NCS_Bernoulli_loss.A1' * P * NCS_Bernoulli_loss.A1 - L_ub*P <= -0.01*eye(NCS.nx);
    cvx_end

    if(cvx_status=="Solved")
        temp_iterations = [lambda_sample,"Solved"];
        iterations=[iterations;temp_iterations];

        lambda_ub = lambda_sample;
    else
        temp_iterations=[lambda_sample,"Infeasible"];
        iterations=[iterations;temp_iterations];

        lambda_lb = lambda_sample;
    end
    delta = lambda_ub - lambda_lb;
end


% then use the maximum possible lambda for further checking

lambda_samples = lambda_ub : 0.02 : 0.8;

iterations = []; lambda_pmax=[];

for i_lambda_samples=1:1:length(lambda_samples)

    delta = 1;
   
    lambda_sample = lambda_samples(i_lambda_samples);   
    lambda = lambda_sample;
    L_ub = 1; L_lb = 1;

    while true

        cvx_clear
        cvx_begin sdp
    
            variable P(2,2) semidefinite
    
            subject to
                NCS_Bernoulli_loss.A0' * P * NCS_Bernoulli_loss.A0 - lambda*P <= -0.01*eye(NCS.nx);
                NCS_Bernoulli_loss.A1' * P * NCS_Bernoulli_loss.A1 - L_ub*P <= -0.01*eye(NCS.nx);

        cvx_end

        if (cvx_status ~= "Solved")
            L_ub = L_ub + 10;
        else
            break;
        end  
    end

    while (delta>0.001)

        L = (L_ub+L_lb)/2;

        % LMIs
        cvx_clear
        cvx_begin sdp
    
            variable P(2,2) semidefinite
    
            subject to
                NCS_Bernoulli_loss.A0' * P * NCS_Bernoulli_loss.A0 - lambda*P <= -0.01*eye(NCS.nx);
                NCS_Bernoulli_loss.A1' * P * NCS_Bernoulli_loss.A1 - L*P <= -0.01*eye(NCS.nx);
        
        cvx_end
    
        if(cvx_status == "Solved")
    
            temp_p_max = -log10(lambda)/(log10(L/lambda));
    
            temp_iterations = [L,lambda,1,temp_p_max];
            iterations=[iterations;temp_iterations];
    
            L_ub = L;             
        else
    
            temp_iterations = [L,lambda,0,0];
            iterations = [iterations;temp_iterations];
    
            L_lb = L;
        end
        delta = L_ub-L_lb;
    end



end
% 
% lambda_pmax

clear i_lambda_samples delta L L_lb L_ub lambda lambda_ub lambda_lb
clear lambda_sample lambda_samples temp_iterations temp_p_max

%% 3.5
p00_samples = 0:0.05:1;
p11_samples = 0:0.05:1;

guaranteed = ones(length(p00_samples),length(p11_samples));

for i = 1:1:length(p00_samples)
    p00 = p00_samples(i);
    
    for j = 1:1:length(p11_samples)
        p11 = p11_samples(j);

        cvx_clear
        cvx_begin sdp

            variable P0(2,2)  semidefinite
            variable P1(2,2)  semidefinite

            subject to
                P0 - (p00 * NCS_Bernoulli_loss.A0' * P0 * NCS_Bernoulli_loss.A0 + (1-p00) * NCS_Bernoulli_loss.A1' * P1 * NCS_Bernoulli_loss.A1) >= 0.001*eye(NCS.nx);
                P1 - (p11 * NCS_Bernoulli_loss.A1' * P1 * NCS_Bernoulli_loss.A1 + (1-p11) * NCS_Bernoulli_loss.A0' * P0 * NCS_Bernoulli_loss.A0) >= 0.001*eye(NCS.nx);

        cvx_end

        if (cvx_status == "Solved")     
            guaranteed(i, j) = 0;
        end
    end
end


contourf(p11_samples, p00_samples, guaranteed)
xlabel("value of p11", 'FontSize', 16);
ylabel("value of p00", 'FontSize', 16);
title("MSS guaranteed area under Gilbert-Elliot models ", 'FontSize', 16)

clear p11 p11_samples p00 p00_samples 
