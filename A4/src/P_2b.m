% 
% rho_set = 1:5:10;
% 
% 
% 
% for i_rho = 1:1:length(rho_set)
% 
%     ADMM_2b.centralized_result = cvx_optval;
%     ADMM_2b.lambda = ones(4, 4);
% 
%     ADMM_2b.decentralized_result = 0;
%     ADMM_2b.error = 100;
%     ADMM_2b.history_error = ADMM_2b.error;
% 
%     i = 0;
% 
%     temp_rho = rho_set(i_rho);
% 
%     aircraft_1.rho = temp_rho;
%     aircraft_2.rho = temp_rho;
%     aircraft_3.rho = temp_rho;
%     aircraft_4.rho = temp_rho;
% 
%     ADMM_2b.terminal = ones(4, 1);
% 
%     subplot(2, 3, i_rho);
% 
%     while(ADMM_2b.error > 0.01)
% 
%         aircraft_1.lambda = ADMM_2b.lambda(1, :);
%         aircraft_2.lambda = ADMM_2b.lambda(2, :);
%         aircraft_3.lambda = ADMM_2b.lambda(3, :);
%         aircraft_4.lambda = ADMM_2b.lambda(4, :);
% 
%         aircraft_1_optimization;
%         aircraft_2_optimization;
%         aircraft_3_optimization;
%         aircraft_4_optimization;
% 
%         ADMM_2b.terminal = aircraft_1.distributed_final_state + 1 / aircraft_1.rho * aircraft_1.lambda';
%         ADMM_2b.terminal = ADMM_2b.terminal + aircraft_2.distributed_final_state + 1 / aircraft_2.rho * aircraft_2.lambda';
%         ADMM_2b.terminal = ADMM_2b.terminal + aircraft_3.distributed_final_state + 1 / aircraft_3.rho * aircraft_3.lambda';
%         ADMM_2b.terminal = ADMM_2b.terminal + aircraft_4.distributed_final_state + 1 / aircraft_4.rho * aircraft_4.lambda';
%         ADMM_2b.terminal = ADMM_2b.terminal / 4;
% 
%         temp_final_state = [aircraft_1.distributed_final_state, aircraft_2.distributed_final_state, aircraft_3.distributed_final_state, aircraft_4.distributed_final_state];
%         temp_final_state = temp_final_state-[ADMM_2b.terminal, ADMM_2b.terminal, ADMM_2b.terminal, ADMM_2b.terminal];
% 
%         ADMM_2b.lambda = ADMM_2b.lambda' + aircraft_1.rho * temp_final_state;
%         ADMM_2b.lambda = ADMM_2b.lambda';
% 
%         aircraft_1.value = (aircraft_1.P * aircraft_1.u(1:aircraft_4.nx_input) + aircraft_1.Q * aircraft_1.initial_state)' * (aircraft_1.P * aircraft_1.u(1:aircraft_4.nx_input) + aircraft_1.Q * aircraft_1.initial_state)..., 
%                           + aircraft_1.u' * aircraft_1.u + x01' * x01;
%         aircraft_2.value = (aircraft_2.P * aircraft_2.u(1:aircraft_4.nx_input) + aircraft_2.Q * aircraft_2.initial_state)' * (aircraft_2.P * aircraft_2.u(1:aircraft_4.nx_input) + aircraft_2.Q * aircraft_2.initial_state)..., 
%                           + aircraft_2.u' * aircraft_2.u + x02' * x02;
%         aircraft_3.value = (aircraft_3.P * aircraft_3.u(1:aircraft_4.nx_input) + aircraft_3.Q * aircraft_3.initial_state)' * (aircraft_3.P * aircraft_3.u(1:aircraft_4.nx_input) + aircraft_3.Q * aircraft_3.initial_state)..., 
%                           + aircraft_3.u' * aircraft_3.u + x03' * x03;
%         aircraft_4.value = (aircraft_4.P * aircraft_4.u(1:aircraft_4.nx_input) + aircraft_4.Q * aircraft_4.initial_state)' * (aircraft_4.P * aircraft_4.u(1:aircraft_4.nx_input) + aircraft_4.Q * aircraft_4.initial_state)..., 
%                           + aircraft_4.u' * aircraft_4.u + x04' * x04;
% 
%         ADMM_2b.decentralized_result = aircraft_1.value + aircraft_2.value + aircraft_3.value + aircraft_4.value;
%         ADMM_2b.error = abs(ADMM_2b.decentralized_result - ADMM_2b.centralized_result) / ADMM_2b.centralized_result;
%         ADMM_2b.history_error = [ADMM_2b.history_error, ADMM_2b.error];
% 
%         i = i + 1;
%         temp_rho
%         ADMM_2b.error
%     end
% 
%     semilogy(ADMM_2b.history_error(2:end));
%     title("rho = " + num2str(temp_rho));
%     xlabel("iterations");
%     ylabel("error");
%     grid on;
% 
% end
% clear temp_final_state

rho_set = 1:2:10;


ADMM_2a.centralized_result = cvx_optval;
figure;
for i_rho = 1:1:length(rho_set)
    
    temp_rho = rho_set(i_rho);
    ADMM_2a.lambda = ones(4, 4);

    
    ADMM_2a.decentralized_result = 0;
    ADMM_2a.error = 100;
    ADMM_2a.history_error = ADMM_2a.error;

    i = 0;

    aircraft_1.rho = temp_rho;
    aircraft_2.rho = temp_rho;
    aircraft_3.rho = temp_rho;
    aircraft_4.rho = temp_rho;

    ADMM_2a.terminal = ones(4, 1);

    aircraft_1.history_objective_error = [];
    aircraft_2.history_objective_error = [];
    aircraft_3.history_objective_error = [];
    aircraft_4.history_objective_error = [];

    while(ADMM_2a.error > 0.005)

        aircraft_1.lambda = ADMM_2a.lambda(1, :);
        aircraft_2.lambda = ADMM_2a.lambda(2, :);
        aircraft_3.lambda = ADMM_2a.lambda(3, :);
        aircraft_4.lambda = ADMM_2a.lambda(4, :);

        aircraft_1_optimization;
        aircraft_2_optimization;
        aircraft_3_optimization;
        aircraft_4_optimization;

        ADMM_2a.terminal = aircraft_1.distributed_final_state + 1 / aircraft_1.rho * aircraft_1.lambda';
        ADMM_2a.terminal = ADMM_2a.terminal + aircraft_2.distributed_final_state + 1 / aircraft_2.rho * aircraft_2.lambda';
        ADMM_2a.terminal = ADMM_2a.terminal + aircraft_3.distributed_final_state + 1 / aircraft_3.rho * aircraft_3.lambda';
        ADMM_2a.terminal = ADMM_2a.terminal + aircraft_4.distributed_final_state + 1 / aircraft_4.rho * aircraft_4.lambda';
        ADMM_2a.terminal = ADMM_2a.terminal / 4;

        temp_final_state = [aircraft_1.distributed_final_state, aircraft_2.distributed_final_state, aircraft_3.distributed_final_state, aircraft_4.distributed_final_state];
        temp_final_state = temp_final_state-[ADMM_2a.terminal, ADMM_2a.terminal, ADMM_2a.terminal, ADMM_2a.terminal];

        ADMM_2a.lambda = ADMM_2a.lambda' + aircraft_1.rho * temp_final_state;
        ADMM_2a.lambda = ADMM_2a.lambda';

        aircraft_1.decentralized_result = (aircraft_1.P * aircraft_1.u + aircraft_1.Q * x01)' * (aircraft_1.P * aircraft_1.u + aircraft_1.Q * x01)...,
                                         + aircraft_1.u' * aircraft_1.u + x01' * x01;
        aircraft_1.error = abs(aircraft_1.decentralized_result-aircraft_1.centralized_result) / aircraft_1.centralized_result;
        aircraft_1.history_objective_error = [aircraft_1.history_objective_error, aircraft_1.error];

        aircraft_2.decentralized_result = (aircraft_2.P * aircraft_2.u + aircraft_2.Q * x02)' * (aircraft_2.P * aircraft_2.u + aircraft_2.Q * x02)...,
                                         + aircraft_2.u' * aircraft_2.u + x02' * x02;
        aircraft_2.error = abs(aircraft_2.decentralized_result-aircraft_2.centralized_result) / aircraft_2.centralized_result;
        aircraft_2.history_objective_error = [aircraft_2.history_objective_error, aircraft_2.error];

        aircraft_3.decentralized_result = (aircraft_3.P * aircraft_3.u + aircraft_3.Q * x03)' * (aircraft_3.P * aircraft_3.u + aircraft_3.Q * x03)...,
                                         + aircraft_3.u' * aircraft_3.u + x03' * x03;
        aircraft_3.error = abs(aircraft_3.decentralized_result-aircraft_3.centralized_result) / aircraft_3.centralized_result;
        aircraft_3.history_objective_error = [aircraft_3.history_objective_error, aircraft_3.error];

        aircraft_4.decentralized_result = (aircraft_4.P * aircraft_4.u + aircraft_4.Q * x04)' * (aircraft_4.P * aircraft_4.u + aircraft_4.Q * x04)...,
                                         + aircraft_4.u' * aircraft_4.u + x04' * x04;
        aircraft_4.error = abs(aircraft_4.decentralized_result-aircraft_4.centralized_result) / aircraft_4.centralized_result;
        aircraft_4.history_objective_error = [aircraft_4.history_objective_error, aircraft_4.error];

        % dual_part = ADMM_master.lambda(1, :) * (aircraft_1.distributed_final_state-aircraft_2.distributed_final_state)...,
        %             + ADMM_master.lambda(2, :) * (aircraft_2.distributed_final_state-aircraft_3.distributed_final_state)...,
        %             + ADMM_master.lambda(3, :) * (aircraft_3.distributed_final_state-aircraft_4.distributed_final_state)..., 
        %             + ADMM_master.lambda(4, :) * (aircraft_4.distributed_final_state-aircraft_1.distributed_final_state);

        dual_part = 0;

        ADMM_2a.decentralized_result = aircraft_1.decentralized_result + aircraft_2.decentralized_result ..., 
                                         + aircraft_3.decentralized_result + aircraft_4.decentralized_result ..., 
                                         + dual_part;


        ADMM_2a.error = abs(ADMM_2a.decentralized_result-ADMM_2a.centralized_result) / ADMM_2a.centralized_result;
        ADMM_2a.history_error = [ADMM_2a.history_error, ADMM_2a.error];

        i = i + 1
        ADMM_2a.error
        
        temp_rho

    end
    
    
    semilogy(ADMM_2a.history_error(2:end));
    
    hold on;
    
end
legend('\rho = 1', '\rho = 3', '\rho = 5', '\rho = 7', '\rho = 9')
title("method error evolution", 'FontSize', 16);
grid on;
xlabel("iterations", 'FontSize', 16);
ylabel("error", 'FontSize', 16);
% ADMM_2a.lambda = ones(4, 4);
% 
% ADMM_2a.centralized_result = cvx_optval;
% ADMM_2a.decentralized_result = 0;
% ADMM_2a.error = 100;
% ADMM_2a.history_error = ADMM_2a.error;
% 
% i = 0;
% 
% aircraft_1.rho = 1;
% aircraft_2.rho = 1;
% aircraft_3.rho = 1;
% aircraft_4.rho = 1;
% 
% ADMM_2a.terminal = ones(4, 1);
% 
% aircraft_1.history_objective_error = [];
% aircraft_2.history_objective_error = [];
% aircraft_3.history_objective_error = [];
% aircraft_4.history_objective_error = [];
% 
% while(ADMM_2a.error > 0.01)
% 
%     aircraft_1.lambda = ADMM_2a.lambda(1, :);
%     aircraft_2.lambda = ADMM_2a.lambda(2, :);
%     aircraft_3.lambda = ADMM_2a.lambda(3, :);
%     aircraft_4.lambda = ADMM_2a.lambda(4, :);
% 
%     aircraft_1_optimization;
%     aircraft_2_optimization;
%     aircraft_3_optimization;
%     aircraft_4_optimization;
% 
%     ADMM_2a.terminal = aircraft_1.distributed_final_state + 1 / aircraft_1.rho * aircraft_1.lambda';
%     ADMM_2a.terminal = ADMM_2a.terminal + aircraft_2.distributed_final_state + 1 / aircraft_2.rho * aircraft_2.lambda';
%     ADMM_2a.terminal = ADMM_2a.terminal + aircraft_3.distributed_final_state + 1 / aircraft_3.rho * aircraft_3.lambda';
%     ADMM_2a.terminal = ADMM_2a.terminal + aircraft_4.distributed_final_state + 1 / aircraft_4.rho * aircraft_4.lambda';
%     ADMM_2a.terminal = ADMM_2a.terminal / 4;
% 
%     temp_final_state = [aircraft_1.distributed_final_state, aircraft_2.distributed_final_state, aircraft_3.distributed_final_state, aircraft_4.distributed_final_state];
%     temp_final_state = temp_final_state-[ADMM_2a.terminal, ADMM_2a.terminal, ADMM_2a.terminal, ADMM_2a.terminal];
% 
%     ADMM_2a.lambda = ADMM_2a.lambda' + aircraft_1.rho * temp_final_state;
%     ADMM_2a.lambda = ADMM_2a.lambda';
% 
%     aircraft_1.decentralized_result = (aircraft_1.P * aircraft_1.u + aircraft_1.Q * x01)' * (aircraft_1.P * aircraft_1.u + aircraft_1.Q * x01)...,
%                                      + aircraft_1.u' * aircraft_1.u + x01' * x01;
%     aircraft_1.error = abs(aircraft_1.decentralized_result-aircraft_1.centralized_result) / aircraft_1.centralized_result;
%     aircraft_1.history_objective_error = [aircraft_1.history_objective_error, aircraft_1.error];
% 
%     aircraft_2.decentralized_result = (aircraft_2.P * aircraft_2.u + aircraft_2.Q * x02)' * (aircraft_2.P * aircraft_2.u + aircraft_2.Q * x02)...,
%                                      + aircraft_2.u' * aircraft_2.u + x02' * x02;
%     aircraft_2.error = abs(aircraft_2.decentralized_result-aircraft_2.centralized_result) / aircraft_2.centralized_result;
%     aircraft_2.history_objective_error = [aircraft_2.history_objective_error, aircraft_2.error];
% 
%     aircraft_3.decentralized_result = (aircraft_3.P * aircraft_3.u + aircraft_3.Q * x03)' * (aircraft_3.P * aircraft_3.u + aircraft_3.Q * x03)...,
%                                      + aircraft_3.u' * aircraft_3.u + x03' * x03;
%     aircraft_3.error = abs(aircraft_3.decentralized_result-aircraft_3.centralized_result) / aircraft_3.centralized_result;
%     aircraft_3.history_objective_error = [aircraft_3.history_objective_error, aircraft_3.error];
% 
%     aircraft_4.decentralized_result = (aircraft_4.P * aircraft_4.u + aircraft_4.Q * x04)' * (aircraft_4.P * aircraft_4.u + aircraft_4.Q * x04)...,
%                                      + aircraft_4.u' * aircraft_4.u + x04' * x04;
%     aircraft_4.error = abs(aircraft_4.decentralized_result-aircraft_4.centralized_result) / aircraft_4.centralized_result;
%     aircraft_4.history_objective_error = [aircraft_4.history_objective_error, aircraft_4.error];
% 
%     % dual_part = ADMM_master.lambda(1, :) * (aircraft_1.distributed_final_state-aircraft_2.distributed_final_state)...,
%     %             + ADMM_master.lambda(2, :) * (aircraft_2.distributed_final_state-aircraft_3.distributed_final_state)...,
%     %             + ADMM_master.lambda(3, :) * (aircraft_3.distributed_final_state-aircraft_4.distributed_final_state)..., 
%     %             + ADMM_master.lambda(4, :) * (aircraft_4.distributed_final_state-aircraft_1.distributed_final_state);
% 
%     dual_part = 0;
% 
%     ADMM_2a.decentralized_result = aircraft_1.decentralized_result + aircraft_2.decentralized_result ..., 
%                                      + aircraft_3.decentralized_result + aircraft_4.decentralized_result ..., 
%                                      + dual_part;
% 
% 
%     ADMM_2a.error = abs(ADMM_2a.decentralized_result-ADMM_2a.centralized_result) / ADMM_2a.centralized_result;
%     ADMM_2a.history_error = [ADMM_2a.history_error, ADMM_2a.error];
%     
%     i = i + 1;
%     ADMM_2a.error
%     i
% 
% end