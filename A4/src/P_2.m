%%%%% Centralized Optimization
cvx_clear

% cvxp = cvx_precision('best');
cvx_begin  sdp

    cvx_solver sedumi

    variables u_1(length_action,1) u_2(length_action,1) u_3(length_action,1) u_4(length_action,1);
    variable x_terminal(4,1);


    minimize ((aircraft_1.P * u_1 + aircraft_1.Q * x01)' * (aircraft_1.P * u_1 + aircraft_1.Q * x01) + (aircraft_2.P *u_2 + aircraft_2.Q * x02)' * (aircraft_2.P *u_2 + aircraft_2.Q * x02) ...,
            + (aircraft_3.P * u_3 + aircraft_3.Q * x03)' * (aircraft_3.P * u_3 + aircraft_3.Q * x03) + (aircraft_4.P *u_4 + aircraft_4.Q * x04)' * (aircraft_4.P *u_4 + aircraft_4.Q * x04) ...,
            + u_1' * u_1 + u_2' * u_2 + u_3' * u_3 + u_4' * u_4 ...,
            + x01' * x01 + x02' * x02 + x03' * x03 + x04' * x04)

    subject to

        u_1 <=  aircraft_1.input_max * ones(length_action, 1);
        u_1 >= -aircraft_1.input_max * ones(length_action, 1);
        u_2 <=  aircraft_2.input_max * ones(length_action, 1);
        u_2 >= -aircraft_2.input_max * ones(length_action, 1);
        u_3 <=  aircraft_3.input_max * ones(length_action, 1);
        u_3 >= -aircraft_3.input_max * ones(length_action, 1);
        u_4 <=  aircraft_4.input_max * ones(length_action, 1);
        u_4 >= -aircraft_4.input_max * ones(length_action, 1);

        aircraft_1.final_transfer * u_1 + A1^5 * x01 - x_terminal == 0;
        aircraft_2.final_transfer * u_2 + A2^5 * x02 - x_terminal == 0;
        aircraft_3.final_transfer * u_3 + A3^5 * x03 - x_terminal == 0;
        aircraft_4.final_transfer * u_4 + A4^5 * x04 - x_terminal == 0;

cvx_end
cvx_precision('best');


aircraft_1.centralized_u = u_1;
aircraft_2.centralized_u = u_2;
aircraft_3.centralized_u = u_3;
aircraft_4.centralized_u = u_4;

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

% centralized state transition
aircraft_1.centralized_state = state_1;
aircraft_2.centralized_state = state_2;
aircraft_3.centralized_state = state_3;
aircraft_4.centralized_state = state_4;

clear i
clear temp_state_1 temp_state_2 temp_state_3 temp_state_4
clear state_1 state_2 state_3 state_4

% centralized final state
aircraft_1.centralized_final_state = aircraft_1.centralized_state(:, end);
aircraft_2.centralized_final_state = aircraft_2.centralized_state(:, end);
aircraft_3.centralized_final_state = aircraft_3.centralized_state(:, end);
aircraft_4.centralized_final_state = aircraft_4.centralized_state(:, end);

% centralized objective value
ADMM_master.centralized_result=cvx_optval;
aircraft_1.centralized_result = (aircraft_1.P * u_1 + aircraft_1.Q * x01)' * (aircraft_1.P * u_1 + aircraft_1.Q * x01) + u_1' * u_1 + x01' * x01;
aircraft_2.centralized_result = (aircraft_2.P * u_2 + aircraft_2.Q * x02)' * (aircraft_2.P * u_2 + aircraft_2.Q * x02) + u_2' * u_2 + x02' * x02;
aircraft_3.centralized_result = (aircraft_3.P * u_3 + aircraft_3.Q * x03)' * (aircraft_3.P * u_3 + aircraft_3.Q * x03) + u_3' * u_3 + x03' * x03;
aircraft_4.centralized_result = (aircraft_4.P * u_4 + aircraft_4.Q * x04)' * (aircraft_4.P * u_4 + aircraft_4.Q * x04) + u_4' * u_4 + x04' * x04;
