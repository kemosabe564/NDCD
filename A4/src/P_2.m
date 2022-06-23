%%%%% Centralized Optimization
cvx_clear

% cvxp = cvx_precision('best');
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
aircraft_1.centralized_result = (aircraft_1.P * u1 + aircraft_1.Q * x01)' * (aircraft_1.P * u1 + aircraft_1.Q * x01) + u1' * u1 + x01' * x01;
aircraft_2.centralized_result = (aircraft_2.P * u2 + aircraft_2.Q * x02)' * (aircraft_2.P * u2 + aircraft_2.Q * x02) + u2' * u2 + x02' * x02;
aircraft_3.centralized_result = (aircraft_3.P * u3 + aircraft_3.Q * x03)' * (aircraft_3.P * u3 + aircraft_3.Q * x03) + u3' * u3 + x03' * x03;
aircraft_4.centralized_result = (aircraft_4.P * u4 + aircraft_4.Q * x04)' * (aircraft_4.P * u4 + aircraft_4.Q * x04) + u4' * u4 + x04' * x04;
