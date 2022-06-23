%%%%% Question 1.4

% figure

% aircraft_1.initial_step=0.001;
% aircraft_2.initial_step=0.001;
% aircraft_3.initial_step=0.001;
% aircraft_4.initial_step=0.1;

% aircraft_1.update_step=aircraft_1.initial_step;
% aircraft_2.update_step=aircraft_2.initial_step;
% aircraft_3.update_step=aircraft_3.initial_step;
% aircraft_4.update_step=aircraft_4.initial_step;


% i=0

% % normal incremental

% incremental_master=struct;
% incremental_master.centralized_result=cvx_optval;
% incremental_master.decentralized_result=0;
% incremental_master.error=1;
% incremental_master.history_objective_error=incremental_master.error;

% aircraft_1.u=zeros(10,1);
% aircraft_2.u=zeros(10,1);
% aircraft_3.u=zeros(10,1);
% aircraft_4.u=zeros(10,1);

% aircraft_1.lambda=ones(4,4);
% aircraft_2.lambda=ones(4,4);
% aircraft_3.lambda=ones(4,4);
% aircraft_4.lambda=ones(4,4);

% incremental_master.lambda=aircraft_1.lambda;

% aircraft_1.history_final_state=[];
% aircraft_2.history_final_state=[];
% aircraft_3.history_final_state=[];
% aircraft_4.history_final_state=[];

% while (incremental_master.error>0.01)  || i<5000

%     aircraft_1.lambda=incremental_master.lambda;
%     temp_lambda=incremental_master.lambda;
%     aircraft_1_optimization;
%     aircraft_1.subgradient=aircraft_1.final_transfer*aircraft_1.u+aircraft_1.A^5*aircraft_1.initial_state;
%     temp_lambda(1,:)=temp_lambda(1,:)+aircraft_1.update_step*aircraft_1.subgradient';
%     temp_lambda(4,:)=temp_lambda(4,:)-aircraft_1.update_step*aircraft_1.subgradient';
%     temp_lambda(find(temp_lambda==0))=0.001;

%     aircraft_2.lambda=temp_lambda;
%     aircraft_2_optimization;
%     aircraft_2.subgradient=aircraft_2.final_transfer*aircraft_2.u+aircraft_2.A^5*aircraft_2.initial_state;   
%     temp_lambda(2,:)=temp_lambda(2,:)+aircraft_2.update_step*aircraft_2.subgradient';
%     temp_lambda(1,:)=temp_lambda(1,:)-aircraft_2.update_step*aircraft_2.subgradient';
%     temp_lambda(find(temp_lambda==0))=0.001;

%     aircraft_3.lambda=temp_lambda;
%     aircraft_3_optimization;
%     aircraft_3.subgradient=aircraft_3.final_transfer*aircraft_3.u+aircraft_3.A^5*aircraft_3.initial_state;
%     temp_lambda(3,:)=temp_lambda(3,:)+aircraft_3.update_step*aircraft_3.subgradient';
%     temp_lambda(2,:)=temp_lambda(2,:)-aircraft_3.update_step*aircraft_3.subgradient';
%     temp_lambda(find(temp_lambda==0))=0.001;

%     aircraft_4.lambda=temp_lambda;
%     aircraft_4_optimization;
%     aircraft_4.subgradient=aircraft_4.final_transfer*aircraft_4.u+aircraft_4.A^5*aircraft_4.initial_state;
%     temp_lambda(4,:)=temp_lambda(4,:)+aircraft_4.update_step*aircraft_4.subgradient';
%     temp_lambda(3,:)=temp_lambda(3,:)-aircraft_4.update_step*aircraft_4.subgradient';
%     temp_lambda(find(temp_lambda==0))=0.001;

%     incremental_master.lambda=temp_lambda;

%     aircraft_1.history_final_state=[aircraft_1.history_final_state,aircraft_1.distributed_final_state];
%     aircraft_2.history_final_state=[aircraft_2.history_final_state,aircraft_2.distributed_final_state];
%     aircraft_3.history_final_state=[aircraft_3.history_final_state,aircraft_3.distributed_final_state];
%     aircraft_4.history_final_state=[aircraft_4.history_final_state,aircraft_4.distributed_final_state];

%     incremental_master.decentralized_result=...,
%                     ( (aircraft_1.P *aircraft_1.u + aircraft_1.Q * x01)' * (aircraft_1.P *aircraft_1.u + aircraft_1.Q * x01)...,
%                     + (aircraft_2.P *aircraft_2.u + aircraft_2.Q * x02)' * (aircraft_2.P *aircraft_2.u + aircraft_2.Q * x02) ...,
%                     + (aircraft_3.P *aircraft_3.u + aircraft_3.Q * x03)' * (aircraft_3.P *aircraft_3.u + aircraft_3.Q * x03) ...,
%                     + (aircraft_4.P *aircraft_4.u + aircraft_4.Q * x04)' * (aircraft_4.P *aircraft_4.u + aircraft_4.Q * x04) ...,
%                     + aircraft_1.u' * aircraft_1.u + aircraft_2.u' * aircraft_2.u + aircraft_3.u' * aircraft_3.u + aircraft_4.u' * aircraft_4.u ...,
%                     + x01'*x01 + x02'*x02 + x03'*x03 +x04'*x04);

%     incremental_master.error=abs(incremental_master.decentralized_result-incremental_master.centralized_result)/incremental_master.centralized_result;
%     incremental_master.history_objective_error=[incremental_master.history_objective_error,incremental_master.error]

%     i=i+1

%     aircraft_1.update_step=aircraft_1.initial_step/i;
%     aircraft_2.update_step=aircraft_2.initial_step/i;
%     aircraft_3.update_step=aircraft_3.initial_step/i;
%     aircraft_4.update_step=aircraft_4.initial_step/i;

%     if(incremental_master.error<0.001)
        
%         input("check");

%     end



% end


% figure
% % plot(incremental_master.history_objective_error);
% semilogy(incremental_master.history_objective_error);
% grid on;
% title("normal incremental");

% normal consensus


% initial_step_set=[0.002,0.002,0.004,0.004,0.008,0.008];
% initial_step_set=[0.002,0.002,0.002,0.004,0.004,0.008];
initial_step_set=[0.001,0.001,0.001,0.002,0.004,0.004];

consensus_step_set=[1,2,5,20,50,100];
% consensus_step_set=[10];

consensus_master=struct;
consensus_master.centralized_result=cvx_optval;
figure('Name','master error history')

for i_consensus_step=1:1:length(consensus_step_set)
% for i_consensus_step=1:1:1
    % i_consensus_step=6

    consensus_master.consensus_step=consensus_step_set(i_consensus_step);

    temp_initial_step=initial_step_set(i_consensus_step);
    aircraft_1.initial_step=temp_initial_step;
    aircraft_2.initial_step=temp_initial_step;
    aircraft_3.initial_step=temp_initial_step;
    aircraft_4.initial_step=temp_initial_step;

    temp_W=W^consensus_master.consensus_step;

    aircraft_1.lambda=ones(4,4);
    aircraft_2.lambda=ones(4,4);
    aircraft_3.lambda=ones(4,4);
    aircraft_4.lambda=ones(4,4);

    aircraft_1.history_lambda=aircraft_1.lambda(1,4);

    consensus_master.decentralized_result=0;
    consensus_master.error=1;
    consensus_master.history_objective_error=consensus_master.error;
    
    i=0;
    aircraft_1.update_step=aircraft_1.initial_step;
    aircraft_2.update_step=aircraft_2.initial_step;
    aircraft_3.update_step=aircraft_3.initial_step;
    aircraft_4.update_step=aircraft_4.initial_step;


    aircraft_1.history_objective_error=[];
    aircraft_2.history_objective_error=[];
    aircraft_3.history_objective_error=[];
    aircraft_4.history_objective_error=[];

    while (consensus_master.error>0.01) 

        aircraft_1_optimization;
        aircraft_2_optimization;
        aircraft_3_optimization;
        aircraft_4_optimization;

        aircraft_1.subgradient=aircraft_1.distributed_final_state;
        aircraft_2.subgradient=aircraft_2.distributed_final_state;
        aircraft_3.subgradient=aircraft_3.distributed_final_state;
        aircraft_4.subgradient=aircraft_4.distributed_final_state;

        temp_lambda=aircraft_1.lambda;
        temp_lambda(1,:)=temp_lambda(1,:)+aircraft_1.update_step*aircraft_1.subgradient';
        temp_lambda(4,:)=temp_lambda(4,:)-aircraft_1.update_step*aircraft_1.subgradient';
        aircraft_1.lambda=temp_lambda;

        temp_lambda=aircraft_2.lambda;
        temp_lambda(2,:)=temp_lambda(2,:)+aircraft_2.update_step*aircraft_2.subgradient';
        temp_lambda(1,:)=temp_lambda(1,:)-aircraft_2.update_step*aircraft_2.subgradient';
        aircraft_2.lambda=temp_lambda;

        temp_lambda=aircraft_3.lambda;
        temp_lambda(3,:)=temp_lambda(3,:)+aircraft_3.update_step*aircraft_3.subgradient';
        temp_lambda(2,:)=temp_lambda(2,:)-aircraft_3.update_step*aircraft_3.subgradient';
        aircraft_3.lambda=temp_lambda;

        temp_lambda=aircraft_4.lambda;
        temp_lambda(4,:)=temp_lambda(4,:)+aircraft_4.update_step*aircraft_4.subgradient';
        temp_lambda(3,:)=temp_lambda(3,:)-aircraft_4.update_step*aircraft_4.subgradient';
        aircraft_4.lambda=temp_lambda;

        temp_lambda_1=temp_W(1,1)*aircraft_1.lambda+temp_W(1,2)*aircraft_2.lambda+temp_W(1,3)*aircraft_3.lambda+temp_W(1,4)*aircraft_4.lambda;
        temp_lambda_2=temp_W(2,1)*aircraft_1.lambda+temp_W(2,2)*aircraft_2.lambda+temp_W(2,3)*aircraft_3.lambda+temp_W(2,4)*aircraft_4.lambda;
        temp_lambda_3=temp_W(3,1)*aircraft_1.lambda+temp_W(3,2)*aircraft_2.lambda+temp_W(3,3)*aircraft_3.lambda+temp_W(3,4)*aircraft_4.lambda;
        temp_lambda_4=temp_W(4,1)*aircraft_1.lambda+temp_W(4,2)*aircraft_2.lambda+temp_W(4,3)*aircraft_3.lambda+temp_W(4,4)*aircraft_4.lambda;
        
        aircraft_1.lambda=temp_lambda_1;
        aircraft_2.lambda=temp_lambda_2;
        aircraft_3.lambda=temp_lambda_3;
        aircraft_4.lambda=temp_lambda_4;

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

%     dual_part=dual_decom_master.lambda(1,:)*(aircraft_1.distributed_final_state-aircraft_2.distributed_final_state)...,
%                 + dual_decom_master.lambda(2,:)*(aircraft_2.distributed_final_state-aircraft_3.distributed_final_state)...,
%                 + dual_decom_master.lambda(3,:)*(aircraft_3.distributed_final_state-aircraft_4.distributed_final_state)...,
%                 + dual_decom_master.lambda(4,:)*(aircraft_4.distributed_final_state-aircraft_1.distributed_final_state);

        dual_part=0;

        consensus_master.decentralized_result=aircraft_1.decentralized_result + aircraft_2.decentralized_result ...,
                                            + aircraft_3.decentralized_result + aircraft_4.decentralized_result...,
                                            + dual_part;


        % consensus_master.decentralized_result=...,
        %                 ( (aircraft_1.P *aircraft_1.u + aircraft_1.Q * x01)' * (aircraft_1.P *aircraft_1.u + aircraft_1.Q * x01)...,
        %                 + (aircraft_2.P *aircraft_2.u + aircraft_2.Q * x02)' * (aircraft_2.P *aircraft_2.u + aircraft_2.Q * x02) ...,
        %                 + (aircraft_3.P *aircraft_3.u + aircraft_3.Q * x03)' * (aircraft_3.P *aircraft_3.u + aircraft_3.Q * x03) ...,
        %                 + (aircraft_4.P *aircraft_4.u + aircraft_4.Q * x04)' * (aircraft_4.P *aircraft_4.u + aircraft_4.Q * x04) ...,
        %                 + aircraft_1.u' * aircraft_1.u + aircraft_2.u' * aircraft_2.u + aircraft_3.u' * aircraft_3.u + aircraft_4.u' * aircraft_4.u ...,
        %                 + x01'*x01 + x02'*x02 + x03'*x03 +x04'*x04);

        consensus_master.error=abs(consensus_master.decentralized_result-consensus_master.centralized_result)/consensus_master.centralized_result;
        consensus_master.error
        consensus_master.history_objective_error=[consensus_master.history_objective_error,consensus_master.error];

        i=i+1
    end
    
    semilogy(consensus_master.history_objective_error(2:end));
    hold on
    
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

legend('\Phi = 1','\Phi = 2','\Phi = 5','\Phi = 20','\Phi = 50','\Phi = 100');
grid on

title('master problem error evolution with different consensus step')
xlabel('iterations', 'FontSize', 16)
ylabel('error', 'FontSize', 16)

figure('Name','aircraft error history')

semilogy(aircraft_1.history_objective_error);
hold on
semilogy(aircraft_2.history_objective_error);
hold on
semilogy(aircraft_3.history_objective_error);
hold on
semilogy(aircraft_4.history_objective_error);
legend('aircraft 1','aircraft 2','aircraft 3','aircraft 4');
grid on

title('error evolution of each aircraft with consensus step is 100')
xlabel('iterations', 'FontSize', 16)
ylabel('error', 'FontSize', 16)
   

figure('Name','aircrafts final state transitions')

subplot(2,2,1)
plot([0:1:Tfinal],aircraft_1.distributed_states');
title('aircraft 1 distributed states')
xlabel('time/t')
ylabel('state value')
grid on

subplot(2,2,2)
plot([0:1:Tfinal],aircraft_2.distributed_states');
title('aircraft 2 distributed states')
xlabel('time/t')
ylabel('state value')
grid on

subplot(2,2,3)
plot([0:1:Tfinal],aircraft_3.distributed_states');
title('aircraft 3 distributed states')
xlabel('time/t')
ylabel('state value')
grid on

subplot(2,2,4)
plot([0:1:Tfinal],aircraft_4.distributed_states');
title('aircraft 4 distributed states')
xlabel('time/t')
ylabel('state value')
grid on

