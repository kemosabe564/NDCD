%%%%% Question 1.2 %%%%%%

%%%%% with diminishing step size 

% use struct to manage the data for each aircrafrt

% 
dual_decom_master.sub_aircrafrt_number = 4;
dual_decom_master.lambda = zeros(4, nx);
dual_decom_master.history_lambda = dual_decom_master.lambda;
dual_decom_master.initial_update_step = 0.1;
dual_decom_master.update_step = dual_decom_master.initial_update_step;

dual_decom_master.centralized_result = cvx_optval;
dual_decom_master.error = 1;
dual_decom_master.history_objective_error = dual_decom_master.error;

i = 0;

aircraft_1.history_objective_error = [];
aircraft_2.history_objective_error = [];
aircraft_3.history_objective_error = [];
aircraft_4.history_objective_error = [];

dual_part = 10;

while dual_decom_master.error > 0.015  

    aircraft_1_optimization
    aircraft_2_optimization
    aircraft_3_optimization
    aircraft_4_optimization

    temp_lambda = dual_decom_master.lambda;
    temp_lambda(1, :) = temp_lambda(1, :) + dual_decom_master.update_step*(aircraft_1.distributed_final_state-aircraft_2.distributed_final_state)';
    temp_lambda(2, :) = temp_lambda(2, :) + dual_decom_master.update_step*(aircraft_2.distributed_final_state-aircraft_3.distributed_final_state)';
    temp_lambda(3, :) = temp_lambda(3, :) + dual_decom_master.update_step*(aircraft_3.distributed_final_state-aircraft_4.distributed_final_state)';
    temp_lambda(4, :) = temp_lambda(4, :) + dual_decom_master.update_step*(aircraft_4.distributed_final_state-aircraft_1.distributed_final_state)';
    dual_decom_master.lambda = temp_lambda;

    dual_decom_master.history_lambda = [dual_decom_master.history_lambda;dual_decom_master.lambda];


    aircraft_1.decentralized_result = (aircraft_1.P *aircraft_1.u  +  aircraft_1.Q * x01)' * (aircraft_1.P *aircraft_1.u  +  aircraft_1.Q * x01)..., 
                                 +  aircraft_1.u' * aircraft_1.u  +  x01'*x01;
    aircraft_1.error = abs(aircraft_1.decentralized_result-aircraft_1.centralized_result)/aircraft_1.centralized_result;
    aircraft_1.history_objective_error = [aircraft_1.history_objective_error, aircraft_1.error];

    aircraft_2.decentralized_result = (aircraft_2.P *aircraft_2.u  +  aircraft_2.Q * x02)' * (aircraft_2.P *aircraft_2.u  +  aircraft_2.Q * x02)..., 
                                 +  aircraft_2.u' * aircraft_2.u  +  x02'*x02;
    aircraft_2.error = abs(aircraft_2.decentralized_result-aircraft_2.centralized_result)/aircraft_2.centralized_result;
    aircraft_2.history_objective_error = [aircraft_2.history_objective_error, aircraft_2.error];

    aircraft_3.decentralized_result = (aircraft_3.P*aircraft_3.u  +  aircraft_3.Q * x03)' * (aircraft_3.P *aircraft_3.u  +  aircraft_3.Q * x03)..., 
                                 +  aircraft_3.u' * aircraft_3.u  +  x03'*x03;
    aircraft_3.error = abs(aircraft_3.decentralized_result-aircraft_3.centralized_result)/aircraft_3.centralized_result;
    aircraft_3.history_objective_error = [aircraft_3.history_objective_error, aircraft_3.error];

    aircraft_4.decentralized_result = (aircraft_4.P *aircraft_4.u  +  aircraft_4.Q * x04)' * (aircraft_4.P *aircraft_4.u  +  aircraft_4.Q * x04)..., 
                                 +  aircraft_4.u' * aircraft_4.u  +  x04'*x04;
    aircraft_4.error = abs(aircraft_4.decentralized_result-aircraft_4.centralized_result)/aircraft_4.centralized_result;
    aircraft_4.history_objective_error = [aircraft_4.history_objective_error, aircraft_4.error];

    % dual_part = dual_decom_master.lambda(1, :)*(aircraft_1.distributed_final_state-aircraft_2.distributed_final_state)..., 
    %              +  dual_decom_master.lambda(2, :)*(aircraft_2.distributed_final_state-aircraft_3.distributed_final_state)..., 
    %              +  dual_decom_master.lambda(3, :)*(aircraft_3.distributed_final_state-aircraft_4.distributed_final_state)..., 
    %              +  dual_decom_master.lambda(4, :)*(aircraft_4.distributed_final_state-aircraft_1.distributed_final_state);

    dual_part = 0;

    dual_decom_master.decentralized_result = aircraft_1.decentralized_result  +  aircraft_2.decentralized_result ..., 
                                         +  aircraft_3.decentralized_result  +  aircraft_4.decentralized_result ..., 
                                         +  dual_part;


    dual_decom_master.error = abs(dual_decom_master.decentralized_result-dual_decom_master.centralized_result)/dual_decom_master.centralized_result;
    dual_decom_master.error
    dual_decom_master.history_objective_error = [dual_decom_master.history_objective_error, dual_decom_master.error];

    i = i + 1

    dual_decom_master.update_step = dual_decom_master.initial_update_step/sqrt(i);
    dual_decom_master.update_step = dual_decom_master.initial_update_step/i;

end

temp_state_1 = aircraft_1.initial_state;
temp_state_2 = aircraft_2.initial_state;
temp_state_3 = aircraft_3.initial_state;
temp_state_4 = aircraft_4.initial_state;

state_1 = [x01];
state_2 = [x02];
state_3 = [x03];
state_4 = [x04];

for i = 1:1:Tfinal

    temp_state_1 = aircraft_1.A*temp_state_1 + aircraft_1.B*aircraft_1.u(2*i-1:2*i);
    state_1 = [state_1, temp_state_1];
    temp_state_2 = aircraft_2.A*temp_state_2 + aircraft_2.B*aircraft_2.u(2*i-1:2*i);
    state_2 = [state_2, temp_state_2];
    temp_state_3 = aircraft_3.A*temp_state_3 + aircraft_3.B*aircraft_3.u(2*i-1:2*i);
    state_3 = [state_3, temp_state_3];
    temp_state_4 = aircraft_4.A*temp_state_4 + aircraft_4.B*aircraft_4.u(2*i-1:2*i);
    state_4 = [state_4, temp_state_4];

end 

aircraft_1.distributed_states = state_1;
aircraft_2.distributed_states = state_2;
aircraft_3.distributed_states = state_3;
aircraft_4.distributed_states = state_4;

%%
figure('Name', 'master problem error evolution')
semilogy(dual_decom_master.history_objective_error(2:end));
title('master problem error evolution')
xlabel('iterations', 'FontSize', 16)
ylabel('error', 'FontSize', 16)
grid on

figure('Name', 'error evolution of each aircraft')

semilogy(aircraft_1.history_objective_error);
hold on
semilogy(aircraft_2.history_objective_error);
hold on
semilogy(aircraft_3.history_objective_error);
hold on
semilogy(aircraft_4.history_objective_error);
legend('aircrafrt 1', 'aircrafrt 2', 'aircrafrt 3', 'aircrafrt 4');
grid on

title('error evolution of each aircraft')
xlabel('iterations', 'FontSize', 16)
ylabel('error', 'FontSize', 16)
   

figure('Name', 'Question 1 Part 2 aircrafrts final state transitions')

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

