%%%%%
%%% p1a
%%%%%
cvx_clear

cvx_begin  sdp

    cvx_solver sedumi

    variables u1(length_action,1) u2(length_action,1) u3(length_action,1) u4(length_action,1);
    variable x_terminal(4,1);


    minimize ((aircraft_1.P * u1 + aircraft_1.Q * x01)' * (aircraft_1.P * u1 + aircraft_1.Q * x01) + (aircraft_2.P *u2 + aircraft_2.Q * x02)' * (aircraft_2.P *u2 + aircraft_2.Q * x02) ...,
            + (aircraft_3.P * u3 + aircraft_3.Q * x03)' * (aircraft_3.P * u3 + aircraft_3.Q * x03) + (aircraft_4.P *u4 + aircraft_4.Q * x04)' * (aircraft_4.P *u4 + aircraft_4.Q * x04) ...,
            + u1' * u1 + u2' * u2 + u3' * u3 + u4' * u4 ...,
            + x01' * x01 + x02' * x02 + x03' * x03 + x04' * x04)

    subject to

        u1 <=  aircraft_1.input_max * ones(length_action, 1);
        u1 >= -aircraft_1.input_max * ones(length_action, 1);
        u2 <=  aircraft_2.input_max * ones(length_action, 1);
        u2 >= -aircraft_2.input_max * ones(length_action, 1);
        u3 <=  aircraft_3.input_max * ones(length_action, 1);
        u3 >= -aircraft_3.input_max * ones(length_action, 1);
        u4 <=  aircraft_4.input_max * ones(length_action, 1);
        u4 >= -aircraft_4.input_max * ones(length_action, 1);

        aircraft_1.matrix * u1 + A1^5 * x01 - x_terminal == 0;
        aircraft_2.matrix * u2 + A2^5 * x02 - x_terminal == 0;
        aircraft_3.matrix * u3 + A3^5 * x03 - x_terminal == 0;
        aircraft_4.matrix * u4 + A4^5 * x04 - x_terminal == 0;

cvx_end
cvx_precision('best');


aircraft_1.centralized_u = u1;
aircraft_2.centralized_u = u2;
aircraft_3.centralized_u = u3;
aircraft_4.centralized_u = u4;

state_1 = aircraft_1.initial_state;
state_2 = aircraft_2.initial_state;
state_3 = aircraft_3.initial_state;
state_4 = aircraft_4.initial_state;

temp_state_1 = state_1;
temp_state_2 = state_2;
temp_state_3 = state_3;
temp_state_4 = state_4;

for i = 1:1:Tfinal

    temp_state_1 = aircraft_1.A * temp_state_1 + aircraft_1.B * aircraft_1.centralized_u(2*i-1 : 2*i);
    temp_state_2 = aircraft_2.A * temp_state_2 + aircraft_2.B * aircraft_2.centralized_u(2*i-1 : 2*i);
    temp_state_3 = aircraft_3.A * temp_state_3 + aircraft_3.B * aircraft_3.centralized_u(2*i-1 : 2*i);
    temp_state_4 = aircraft_4.A * temp_state_4 + aircraft_4.B * aircraft_4.centralized_u(2*i-1 : 2*i);

    state_1 = [state_1, temp_state_1];
    state_2 = [state_2, temp_state_2];
    state_3 = [state_3, temp_state_3];
    state_4 = [state_4, temp_state_4];
end 

aircraft_1.centralized_state = state_1;
aircraft_2.centralized_state = state_2;
aircraft_3.centralized_state = state_3;
aircraft_4.centralized_state = state_4;

clear i
clear temp_state_1 temp_state_2 temp_state_3 temp_state_4
clear state_1 state_2 state_3 state_4

aircraft_1.centralized_final_state = aircraft_1.centralized_state(:, end);
aircraft_2.centralized_final_state = aircraft_2.centralized_state(:, end);
aircraft_3.centralized_final_state = aircraft_3.centralized_state(:, end);
aircraft_4.centralized_final_state = aircraft_4.centralized_state(:, end);

decompose_problem.centralized_result = cvx_optval;
aircraft_1.centralized_result = (aircraft_1.P * u1 + aircraft_1.Q * x01)' * (aircraft_1.P * u1 + aircraft_1.Q * x01) + u1' * u1 + x01' * x01;
aircraft_2.centralized_result = (aircraft_2.P * u2 + aircraft_2.Q * x02)' * (aircraft_2.P * u2 + aircraft_2.Q * x02) + u2' * u2 + x02' * x02;
aircraft_3.centralized_result = (aircraft_3.P * u3 + aircraft_3.Q * x03)' * (aircraft_3.P * u3 + aircraft_3.Q * x03) + u3' * u3 + x03' * x03;
aircraft_4.centralized_result = (aircraft_4.P * u4 + aircraft_4.Q * x04)' * (aircraft_4.P * u4 + aircraft_4.Q * x04) + u4' * u4 + x04' * x04;


decompose_problem.sub_aircraft_number = 4;
decompose_problem.lambda = zeros(4, nx);
decompose_problem.history_lambda = decompose_problem.lambda;
decompose_problem.update_step = 0.002;

decompose_problem.centralized_result = cvx_optval;
decompose_problem.error = 1;
decompose_problem.master_error = decompose_problem.error;

decompose_problem.x_terminal = x_terminal;

i = 0;

aircraft_1.sub_error = [];
aircraft_2.sub_error = [];
aircraft_3.sub_error = [];
aircraft_4.sub_error = [];

while decompose_problem.error > 0.01 

    aircraft_1_optimization
    aircraft_2_optimization
    aircraft_3_optimization
    aircraft_4_optimization

    temp_lambda = decompose_problem.lambda;
    temp_lambda(1,:) = temp_lambda(1, :) + decompose_problem.update_step * (aircraft_1.distributed_final_state - aircraft_2.distributed_final_state)';
    temp_lambda(2,:) = temp_lambda(2, :) + decompose_problem.update_step * (aircraft_2.distributed_final_state - aircraft_3.distributed_final_state)';
    temp_lambda(3,:) = temp_lambda(3, :) + decompose_problem.update_step * (aircraft_3.distributed_final_state - aircraft_4.distributed_final_state)';
    temp_lambda(4,:) = temp_lambda(4, :) + decompose_problem.update_step * (aircraft_4.distributed_final_state - aircraft_1.distributed_final_state)';

    decompose_problem.lambda = temp_lambda;
    decompose_problem.history_lambda = [decompose_problem.history_lambda; decompose_problem.lambda];


    aircraft_1.decentralized_result = (aircraft_1.P * aircraft_1.u + aircraft_1.Q * x01)' * (aircraft_1.P * aircraft_1.u + aircraft_1.Q * x01)...,
                                     + aircraft_1.u' * aircraft_1.u + x01' * x01;
    aircraft_1.error = abs(aircraft_1.decentralized_result - aircraft_1.centralized_result) / aircraft_1.centralized_result;
    aircraft_1.sub_error = [aircraft_1.sub_error, aircraft_1.error];

    aircraft_2.decentralized_result = (aircraft_2.P * aircraft_2.u + aircraft_2.Q * x02)' * (aircraft_2.P * aircraft_2.u + aircraft_2.Q * x02)...,
                                     + aircraft_2.u' * aircraft_2.u + x02' * x02;
    aircraft_2.error = abs(aircraft_2.decentralized_result - aircraft_2.centralized_result) / aircraft_2.centralized_result;
    aircraft_2.sub_error = [aircraft_2.sub_error, aircraft_2.error];

    aircraft_3.decentralized_result = (aircraft_3.P * aircraft_3.u + aircraft_3.Q * x03)' * (aircraft_3.P *aircraft_3.u + aircraft_3.Q * x03)...,
                                     + aircraft_3.u' * aircraft_3.u + x03' * x03;
    aircraft_3.error = abs(aircraft_3.decentralized_result - aircraft_3.centralized_result) / aircraft_3.centralized_result;
    aircraft_3.sub_error = [aircraft_3.sub_error, aircraft_3.error];

    aircraft_4.decentralized_result = (aircraft_4.P * aircraft_4.u + aircraft_4.Q * x04)' * (aircraft_4.P *aircraft_4.u + aircraft_4.Q * x04)...,
                                     + aircraft_4.u' * aircraft_4.u + x04' * x04;
    aircraft_4.error = abs(aircraft_4.decentralized_result - aircraft_4.centralized_result) / aircraft_4.centralized_result;
    aircraft_4.sub_error = [aircraft_4.sub_error, aircraft_4.error];

    dual_part = 0;

    decompose_problem.decentralized_result = aircraft_1.decentralized_result + aircraft_2.decentralized_result ...,
                                           + aircraft_3.decentralized_result + aircraft_4.decentralized_result...,
                                           + dual_part;

    decompose_problem.error = abs(decompose_problem.decentralized_result - decompose_problem.centralized_result) / decompose_problem.centralized_result;
    decompose_problem.master_error = [decompose_problem.master_error, decompose_problem.error];

    i = i + 1
end

temp_state_1 = aircraft_1.initial_state;
temp_state_2 = aircraft_2.initial_state;
temp_state_3 = aircraft_3.initial_state;
temp_state_4 = aircraft_4.initial_state;

state_1 = x01;
state_2 = x02;
state_3 = x03;
state_4 = x04;

for i=1:1:Tfinal

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

fixed_step_history_objective_error = decompose_problem.master_error;
%% plot
figure('Name','P1 centralized states')

subplot(2, 2, 1)
plot(T_span, aircraft_1.centralized_state');
grid on;
title("aircraft 1 centralized states")
xlabel("time/t")
ylabel("state value")

subplot(2, 2, 2)
plot(T_span, aircraft_1.distributed_states');
grid on;
title("aircraft 1 distributed states")
xlabel("time/t/t")
ylabel("state value")

subplot(2, 2, 3)
plot(T_span, aircraft_2.centralized_state');
grid on;
title("aircraft 2 centralized states")
xlabel("time/t")
ylabel("state value")

subplot(2, 2, 4)
plot(T_span, aircraft_2.distributed_states');
grid on;
title("aircraft 2 distributed states")
xlabel("time/t")
ylabel("state value")

figure('Name',' aircrafts 3&4 final evolution')

subplot(2,2,1)
plot(T_span, aircraft_3.centralized_state');
title('aircraft 3 centralized states')
xlabel('time/t')
ylabel('state value')
grid on

subplot(2,2,2)
plot(T_span, aircraft_3.distributed_states');
title('aircraft 3 distributed states')
xlabel('time/t')
ylabel('state value')
grid on

subplot(2,2,3)
plot(T_span, aircraft_4.centralized_state');
title('aircraft 4 centralized states')
xlabel('time/t')
ylabel('state value')
grid on

subplot(2,2,4)
plot(T_span, aircraft_4.distributed_states');
title('aircraft 4 distributed states')
xlabel('time/t')
ylabel('state value')
grid on
   

figure('Name', 'master error history')
semilogy(decompose_problem.master_error(2 : end));
title('master problem error evolution', 'FontSize', 16)
xlabel('iterations', 'FontSize', 16)
ylabel('error', 'FontSize', 16)
grid on

figure('Name','aircraft error history')

semilogy(aircraft_1.sub_error);
hold on
semilogy(aircraft_2.sub_error);
hold on
semilogy(aircraft_3.sub_error);
hold on
semilogy(aircraft_4.sub_error);
legend('aircraft 1', 'aircraft 2', 'aircraft 3', 'aircraft 4');
grid on

title('error evolution of each aircraft', 'FontSize', 16)
xlabel('iterations', 'FontSize', 16)
ylabel('error', 'FontSize', 16)

