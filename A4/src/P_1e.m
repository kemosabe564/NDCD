%%%%% Question 1.5


% Centralized Optimization
cvx_clear

cvxp = cvx_precision( 'best' );
cvx_begin  sdp

    cvx_solver Gurobi

    variables u_1(length_action,1) u_2(length_action,1) u_3(length_action,1) u_4(length_action,1);
    variable x_terminal(4,1);


    minimize ((aircraft_1.P *u_1 + aircraft_1.Q * x01)' * (aircraft_1.P *u_1 + aircraft_1.Q * x01) + (aircraft_2.P *u_2 + aircraft_2.Q * x02)' * (aircraft_2.P *u_2 + aircraft_2.Q * x02) ...,
            + (aircraft_3.P *u_3 + aircraft_3.Q * x03)' * (aircraft_3.P *u_3 + aircraft_3.Q * x03) + (aircraft_4.P *u_4 + aircraft_4.Q * x04)' * (aircraft_4.P *u_4+ aircraft_4.Q * x04) ...,
            + u_1' * u_1 + u_2' * u_2 + u_3' * u_3 + u_4' * u_4 ...,
            + x01'*x01 + x02'*x02 +x03'*x03 +x04'*x04)

    subject to

        sum_square(u_1)<=umax^2;
        sum_square(u_2)<=umax^2;
        sum_square(u_3)<=umax^2;
        sum_square(u_4)<=umax^2;

        final_transfer_1*u_1+A1^5*x01-x_terminal==0;
        final_transfer_2*u_2+A2^5*x02-x_terminal==0;
        final_transfer_3*u_3+A3^5*x03-x_terminal==0;
        final_transfer_4*u_4+A4^5*x04-x_terminal==0;

cvx_end
cvx_precision( cvxp );

aircraft_1.centralized_u=u_1;
aircraft_2.centralized_u=u_2;
aircraft_3.centralized_u=u_3;
aircraft_4.centralized_u=u_4;

state_1=aircraft_1.initial_state;
state_2=aircraft_2.initial_state;
state_3=aircraft_3.initial_state;
state_4=aircraft_4.initial_state;

temp_state_1=aircraft_1.initial_state;
temp_state_2=aircraft_2.initial_state;
temp_state_3=aircraft_3.initial_state;
temp_state_4=aircraft_4.initial_state;

for i=1:1:Tfinal

    temp_state_1=aircraft_1.A*temp_state_1+aircraft_1.B*aircraft_1.centralized_u(2*i-1:2*i);
    state_1=[state_1,temp_state_1];
    temp_state_2=aircraft_2.A*temp_state_2+aircraft_2.B*aircraft_2.centralized_u(2*i-1:2*i);
    state_2=[state_2,temp_state_2];
    temp_state_3=aircraft_3.A*temp_state_3+aircraft_3.B*aircraft_3.centralized_u(2*i-1:2*i);
    state_3=[state_3,temp_state_3];
    temp_state_4=aircraft_4.A*temp_state_4+aircraft_4.B*aircraft_4.centralized_u(2*i-1:2*i);
    state_4=[state_4,temp_state_4];

end 

aircraft_1.centralized_state=state_1;
aircraft_2.centralized_state=state_2;
aircraft_3.centralized_state=state_3;
aircraft_4.centralized_state=state_4;

aircraft_1.centralized_final_state=aircraft_1.centralized_state(:,end);
aircraft_2.centralized_final_state=aircraft_2.centralized_state(:,end);
aircraft_3.centralized_final_state=aircraft_3.centralized_state(:,end);
aircraft_4.centralized_final_state=aircraft_4.centralized_state(:,end);

dual_decom_master.centralized_result=cvx_optval;


% decentralized solution

% use struct to manage the data for each aircraft

dual_decom_master=struct;

dual_decom_master.sub_aircraft_number=4;
dual_decom_master.lambda=zeros(4,length_state);
dual_decom_master.history_lambda=dual_decom_master.lambda;
dual_decom_master.initial_step=0.001;
dual_decom_master.update_step=dual_decom_master.initial_step;

dual_decom_master.centralized_result=cvx_optval;
dual_decom_master.error=1;
dual_decom_master.history_error=dual_decom_master.error;

i=0

aircraft_1.history_objective_error=[];
aircraft_2.history_objective_error=[];
aircraft_3.history_objective_error=[];
aircraft_4.history_objective_error=[];

while dual_decom_master.error>0.01 

    aircraft_1_optimization
    aircraft_2_optimization
    aircraft_3_optimization
    aircraft_4_optimization

    temp_lambda=dual_decom_master.lambda;
    temp_lambda(1,:)=temp_lambda(1,:)+dual_decom_master.update_step*(aircraft_1.distributed_final_state-aircraft_2.distributed_final_state)';
    temp_lambda(2,:)=temp_lambda(2,:)+dual_decom_master.update_step*(aircraft_2.distributed_final_state-aircraft_3.distributed_final_state)';
    temp_lambda(3,:)=temp_lambda(3,:)+dual_decom_master.update_step*(aircraft_3.distributed_final_state-aircraft_4.distributed_final_state)';
    temp_lambda(4,:)=temp_lambda(4,:)+dual_decom_master.update_step*(aircraft_4.distributed_final_state-aircraft_1.distributed_final_state)';
    dual_decom_master.lambda=temp_lambda;

    dual_decom_master.history_lambda=[dual_decom_master.history_lambda;dual_decom_master.lambda];

    aircraft_1.decentralized_result=(aircraft_1.P *aircraft_1.u + aircraft_1.Q * x01)' * (aircraft_1.P *aircraft_1.u + aircraft_1.Q * x01)...,
                                + aircraft_1.u' * aircraft_1.u + x01'*x01;
    aircraft_1.error=abs(aircraft_1.decentralized_result-aircraft_1.centralized_result)/aircraft_1.centralized_result;
    aircraft_1.history_objective_error=[aircraft_1.history_objective_error,aircraft_1.error];

    aircraft_2.decentralized_result=(aircraft_2.P *aircraft_2.u + aircraft_2.Q * x02)' * (aircraft_2.P *aircraft_2.u + aircraft_2.Q * x02)...,
            + aircraft_2.u' * aircraft_2.u + x02'*x02;
    aircraft_2.error=abs(aircraft_2.decentralized_result-aircraft_2.centralized_result)/aircraft_2.centralized_result;
    aircraft_2.history_objective_error=[aircraft_2.history_objective_error,aircraft_2.error];

    aircraft_3.decentralized_result=(aircraft_3.P*aircraft_3.u + aircraft_3.Q * x03)' * (aircraft_3.P *aircraft_3.u + aircraft_3.Q * x03)...,
            + aircraft_3.u' * aircraft_3.u + x03'*x03;
    aircraft_3.error=abs(aircraft_3.decentralized_result-aircraft_3.centralized_result)/aircraft_3.centralized_result;
    aircraft_3.history_objective_error=[aircraft_3.history_objective_error,aircraft_3.error];

    aircraft_4.decentralized_result=(aircraft_4.P *aircraft_4.u + aircraft_4.Q * x04)' * (aircraft_4.P *aircraft_4.u + aircraft_4.Q * x04)...,
            + aircraft_4.u' * aircraft_4.u + x04'*x04;
    aircraft_4.error=abs(aircraft_4.decentralized_result-aircraft_4.centralized_result)/aircraft_4.centralized_result;
    aircraft_4.history_objective_error=[aircraft_4.history_objective_error,aircraft_4.error];

    dual_part=0;

    dual_decom_master.decentralized_result=aircraft_1.decentralized_result + aircraft_2.decentralized_result ...,
                    + aircraft_3.decentralized_result + aircraft_4.decentralized_result...,
                    + dual_part;


    dual_decom_master.error=abs(dual_decom_master.decentralized_result-dual_decom_master.centralized_result)/dual_decom_master.centralized_result
    % dual_decom_master.error
    dual_decom_master.history_error=[dual_decom_master.history_error,dual_decom_master.error];

    i=i+1

    dual_decom_master.update_step=dual_decom_master.initial_step/sqrt(i);

end


% figure
% plot(dual_decom_master.history_error(2:end));
   

% Primitive Nesto Accelerrated

Nesto_dual_decom_master=struct;

Nesto_dual_decom_master.sub_aircraft_number=4;
Nesto_dual_decom_master.lambda=zeros(4,length_state);
Nesto_dual_decom_master.history_lambda=Nesto_dual_decom_master.lambda;
Nesto_dual_decom_master.initial_update_step=0.001;
Nesto_dual_decom_master.update_step=Nesto_dual_decom_master.initial_update_step;

Nesto_dual_decom_master.centralized_result=cvx_optval;
Nesto_dual_decom_master.error=1;
Nesto_dual_decom_master.history_objective_error=Nesto_dual_decom_master.error;

i=0

aircraft_1.history_objective_error=[];
aircraft_2.history_objective_error=[];
aircraft_3.history_objective_error=[];
aircraft_4.history_objective_error=[];

previous_temp_lambda=zeros(4,4);

dual_decom_master.lambda=zeros(4,4);

while Nesto_dual_decom_master.error>0.01  

    aircraft_1_optimization
    aircraft_2_optimization
    aircraft_3_optimization
    aircraft_4_optimization

    temp_lambda=Nesto_dual_decom_master.lambda;
    temp_lambda(1,:)=temp_lambda(1,:)+Nesto_dual_decom_master.update_step*(aircraft_1.distributed_final_state-aircraft_2.distributed_final_state)';
    temp_lambda(2,:)=temp_lambda(2,:)+Nesto_dual_decom_master.update_step*(aircraft_2.distributed_final_state-aircraft_3.distributed_final_state)';
    temp_lambda(3,:)=temp_lambda(3,:)+Nesto_dual_decom_master.update_step*(aircraft_3.distributed_final_state-aircraft_4.distributed_final_state)';
    temp_lambda(4,:)=temp_lambda(4,:)+Nesto_dual_decom_master.update_step*(aircraft_4.distributed_final_state-aircraft_1.distributed_final_state)';
    
    Nesto_dual_decom_master.lambda=temp_lambda + (i+1)/(i+4)* (temp_lambda-previous_temp_lambda);

    Nesto_dual_decom_master.history_lambda=[Nesto_dual_decom_master.history_lambda;Nesto_dual_decom_master.lambda];

    aircraft_1.decentralized_result=(aircraft_1.P *aircraft_1.u + aircraft_1.Q * x01)' * (aircraft_1.P *aircraft_1.u + aircraft_1.Q * x01)...,
                                + aircraft_1.u' * aircraft_1.u + x01'*x01;
    aircraft_1.error=abs(aircraft_1.decentralized_result-aircraft_1.centralized_result)/aircraft_1.centralized_result;
    aircraft_1.history_objective_error=[aircraft_1.history_objective_error,aircraft_1.error];

    aircraft_2.decentralized_result=(aircraft_2.P *aircraft_2.u + aircraft_2.Q * x02)' * (aircraft_2.P *aircraft_2.u + aircraft_2.Q * x02)...,
                                + aircraft_2.u' * aircraft_2.u + x02'*x02;
    aircraft_2.error=abs(aircraft_2.decentralized_result-aircraft_2.centralized_result)/aircraft_2.centralized_result;
    aircraft_2.history_objective_error=[aircraft_2.history_objective_error,aircraft_2.error];

    aircraft_3.decentralized_result=(aircraft_3.P*aircraft_3.u + aircraft_3.Q * x03)' * (aircraft_3.P *aircraft_3.u + aircraft_3.Q * x03)...,
                                + aircraft_3.u' * aircraft_3.u + x03'*x03;
    aircraft_3.error=abs(aircraft_3.decentralized_result-aircraft_3.centralized_result)/aircraft_3.centralized_result;
    aircraft_3.history_objective_error=[aircraft_3.history_objective_error,aircraft_3.error];

    aircraft_4.decentralized_result=(aircraft_4.P *aircraft_4.u + aircraft_4.Q * x04)' * (aircraft_4.P *aircraft_4.u + aircraft_4.Q * x04)...,
                                + aircraft_4.u' * aircraft_4.u + x04'*x04;
    aircraft_4.error=abs(aircraft_4.decentralized_result-aircraft_4.centralized_result)/aircraft_4.centralized_result;
    aircraft_4.history_objective_error=[aircraft_4.history_objective_error,aircraft_4.error];

    dual_part=Nesto_dual_decom_master.lambda(1,:)*(aircraft_1.distributed_final_state-aircraft_2.distributed_final_state)...,
                + Nesto_dual_decom_master.lambda(2,:)*(aircraft_2.distributed_final_state-aircraft_3.distributed_final_state)...,
                + Nesto_dual_decom_master.lambda(3,:)*(aircraft_3.distributed_final_state-aircraft_4.distributed_final_state)...,
                + Nesto_dual_decom_master.lambda(4,:)*(aircraft_4.distributed_final_state-aircraft_1.distributed_final_state);

    % dual_part=0;

    Nesto_dual_decom_master.decentralized_result=aircraft_1.decentralized_result + aircraft_2.decentralized_result ...,
                                        + aircraft_3.decentralized_result + aircraft_4.decentralized_result;
                                        % + dual_part;


    Nesto_dual_decom_master.error=abs(Nesto_dual_decom_master.decentralized_result-Nesto_dual_decom_master.centralized_result)/Nesto_dual_decom_master.centralized_result
    Nesto_dual_decom_master.error
    Nesto_dual_decom_master.history_objective_error=[Nesto_dual_decom_master.history_objective_error,Nesto_dual_decom_master.error];

    i=i+1

    previous_temp_lambda=temp_lambda;

    dual_decom_master.lambda=temp_lambda;

    % Nesto_dual_decom_master.update_step=Nesto_dual_decom_master.initial_update_step/sqrt(i);
    % Nesto_dual_decom_master.update_step=Nesto_dual_decom_master.initial_update_step/i;

end

temp_state_1=aircraft_1.initial_state;
temp_state_2=aircraft_2.initial_state;
temp_state_3=aircraft_3.initial_state;
temp_state_4=aircraft_4.initial_state;

state_1=[x01];
state_2=[x02];
state_3=[x03];
state_4=[x04];

for i=1:1:Tfinal

    temp_state_1=aircraft_1.A*temp_state_1+aircraft_1.B*aircraft_1.u(2*i-1:2*i);
    state_1=[state_1,temp_state_1];
    temp_state_2=aircraft_2.A*temp_state_2+aircraft_2.B*aircraft_2.u(2*i-1:2*i);
    state_2=[state_2,temp_state_2];
    temp_state_3=aircraft_3.A*temp_state_3+aircraft_3.B*aircraft_3.u(2*i-1:2*i);
    state_3=[state_3,temp_state_3];
    temp_state_4=aircraft_4.A*temp_state_4+aircraft_4.B*aircraft_4.u(2*i-1:2*i);
    state_4=[state_4,temp_state_4];

end 

aircraft_1.distributed_states=state_1;
aircraft_2.distributed_states=state_2;
aircraft_3.distributed_states=state_3;
aircraft_4.distributed_states=state_4;

figure('Name','Question 1 Part 3 master error history')
semilogy(Nesto_dual_decom_master.history_objective_error(2:end));
title('Question 1 Part 3 Normal Nestorev master error history')
xlabel('iterations')
ylabel('error')
grid on

figure('Name','Question 1 Part 3 Normal Nestorev aircraft error history')

semilogy(aircraft_1.history_objective_error);
hold on
semilogy(aircraft_2.history_objective_error);
hold on
semilogy(aircraft_3.history_objective_error);
hold on
semilogy(aircraft_4.history_objective_error);
legend('aircraft 1','aircraft 2','aircraft 3','aircraft 4');
grid on

title('aircraft error history')
xlabel('iterations')
ylabel('error')



figure('Name','Question 1 Part 3 Normal Nestorev aircrafts final state transitions')

subplot(2,2,1)
plot([0:1:Tfinal],aircraft_1.distributed_states');
title('Question 1 Part 3 Normal Nestorev aircraft 1 distributed states')
xlabel('time')
ylabel('states')
grid on

subplot(2,2,2)
plot([0:1:Tfinal],aircraft_2.distributed_states');
title('Question 1 Part 3 Normal Nestorev aircraft 2 distributed states')
xlabel('time')
ylabel('states')
grid on

subplot(2,2,3)
plot([0:1:Tfinal],aircraft_3.distributed_states');
title('Question 1 Part 3 Normal Nestorev aircraft 3 distributed states')
xlabel('time')
ylabel('states')
grid on

subplot(2,2,4)
plot([0:1:Tfinal],aircraft_4.distributed_states');
title('Question 1 Part 3 Normal Nestorev aircraft 4 distributed states')
xlabel('time')
ylabel('states')
grid on


