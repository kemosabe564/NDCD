
ADMM_2a.lambda = ones(4, 4);

ADMM_2a.centralized_result = cvx_optval;
ADMM_2a.decentralized_result = 0;
ADMM_2a.error = 100;
ADMM_2a.history_error = ADMM_2a.error;

i = 0;

aircraft_1.rho = 1;
aircraft_2.rho = 1;
aircraft_3.rho = 1;
aircraft_4.rho = 1;

ADMM_2a.terminal = ones(4, 1);

aircraft_1.history_objective_error = [];
aircraft_2.history_objective_error = [];
aircraft_3.history_objective_error = [];
aircraft_4.history_objective_error = [];

while(ADMM_2a.error > 0.01)

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
    
    i = i + 1;
    ADMM_2a.error
    i

end

temp_state_1 = aircraft_1.initial_state;
temp_state_2 = aircraft_2.initial_state;
temp_state_3 = aircraft_3.initial_state;
temp_state_4 = aircraft_4.initial_state;

state_1 = x01;
state_2 = x02;
state_3 = x03;
state_4 = x04;

for i = 1:1:Tfinal

    temp_state_1 = aircraft_1.A * temp_state_1 + aircraft_1.B * aircraft_1.u(2*i-1 : 2*i);    
    temp_state_2 = aircraft_2.A * temp_state_2 + aircraft_2.B * aircraft_2.u(2*i-1 : 2*i);
    temp_state_3 = aircraft_3.A * temp_state_3 + aircraft_3.B * aircraft_3.u(2*i-1 : 2*i);
    temp_state_4 = aircraft_4.A * temp_state_4 + aircraft_4.B * aircraft_4.u(2*i-1 : 2*i);
    
    state_1 = [state_1, temp_state_1];
    state_2 = [state_2, temp_state_2];    
    state_3 = [state_3, temp_state_3];    
    state_4 = [state_4, temp_state_4];

end 

aircraft_1.distributed_states = state_1;
aircraft_2.distributed_states = state_2;
aircraft_3.distributed_states = state_3;
aircraft_4.distributed_states = state_4;

clear i
clear temp_final_state
clear state_1 state_2 state_3 state_4
clear temp_state_1 temp_state_2 temp_state_3 temp_state_4

figure('Name', 'ADMM method error evolution')
semilogy(ADMM_2a.history_error(2:end))
title('ADMM method error evolution', 'FontSize', 16)
ylabel('error', 'FontSize', 16)
xlabel('iterations', 'FontSize', 16)
grid on


figure('Name', 'aircraft error evolution')

semilogy(aircraft_1.history_objective_error);
hold on
semilogy(aircraft_2.history_objective_error);
hold on
semilogy(aircraft_3.history_objective_error);
hold on
semilogy(aircraft_4.history_objective_error);
legend('aircraft 1', 'aircraft 2', 'aircraft 3', 'aircraft 4');
grid on

title('aircraft error evolution', 'FontSize', 16)
xlabel('iterations', 'FontSize', 16)
ylabel('error', 'FontSize', 16)
   

figure('Name', 'aircrafts final state transitions')

subplot(2, 2, 1)
plot(T_span, aircraft_1.distributed_states');
title('aircraft 1 distributed states')
xlabel('time/t')
ylabel('state value')
grid on

subplot(2, 2, 2)
plot(T_span, aircraft_2.distributed_states');
title('aircraft 2 distributed states')
xlabel('time/t')
ylabel('state value')
grid on

subplot(2, 2, 3)
plot(T_span, aircraft_3.distributed_states');
title('aircraft 3 distributed states')
xlabel('time/t')
ylabel('state value')
grid on

subplot(2, 2, 4)
plot(T_span, aircraft_4.distributed_states');
title('aircraft 4 distributed states')
xlabel('time/t')
ylabel('state value')
grid on