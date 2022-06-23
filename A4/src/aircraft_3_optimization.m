%%%%%  aircraft 3

%%%%% aircraft 3 for Question 1a

if (Index=="1a" || Index=="1b")

        aircraft_3.options=optimoptions('quadprog','Display','off');

        aircraft_3.H=aircraft_3.P'*aircraft_3.P;
        aircraft_3.H=aircraft_3.H+eye(length_action);
        aircraft_3.f=aircraft_3.initial_state'*aircraft_3.Q'*aircraft_3.P*2;
        aircraft_3.f=aircraft_3.f+(dual_decom_master.lambda(3,:)-dual_decom_master.lambda(2,:))*aircraft_3.final_transfer;
        aircraft_3.f=aircraft_3.f';


        aircraft_3.u_lb=-aircraft_3.input_max*ones(length_action,1);
        aircraft_3.u_ub=aircraft_3.input_max*ones(length_action,1);
        

        [u_3,fval,exitflag]=quadprog(2*aircraft_3.H,aircraft_3.f,[],[],[],[],aircraft_3.u_lb,aircraft_3.u_ub,[],aircraft_3.options);


%%%%% aircraft 3 for Question 1c

elseif (Index=="1c" )

        aircraft_3.options=optimoptions('quadprog','Display','off');

        aircraft_3.H=aircraft_3.P'*aircraft_3.P;
        aircraft_3.H=aircraft_3.H+eye(length_action);
        aircraft_3.f=aircraft_3.initial_state'*aircraft_3.Q'*aircraft_3.P*2;
        aircraft_3.f=aircraft_3.f+(Nesto_dual_decom_master.lambda(3,:)-Nesto_dual_decom_master.lambda(2,:))*aircraft_3.final_transfer;
        aircraft_3.f=aircraft_3.f';

        aircraft_3.u_lb=-aircraft_3.input_max*ones(length_action,1);
        aircraft_3.u_ub=aircraft_3.input_max*ones(length_action,1);
        

        [u_3,fval,exitflag]=quadprog(2*aircraft_3.H,aircraft_3.f,[],[],[],[],aircraft_3.u_lb,aircraft_3.u_ub,[],aircraft_3.options);


%%%%% aircraft 3 for Question 1d

elseif (Index=="1d")

        aircraft_3.options=optimoptions('quadprog','Display','off');

        aircraft_3.H=aircraft_3.P'*aircraft_3.P;
        aircraft_3.H=aircraft_3.H+eye(length_action);
        aircraft_3.f=aircraft_3.initial_state'*aircraft_3.Q'*aircraft_3.P*2;
        aircraft_3.f=aircraft_3.f+(aircraft_3.lambda(3,:)-aircraft_3.lambda(2,:))*aircraft_3.final_transfer;
        aircraft_3.f=aircraft_3.f';


        aircraft_3.u_lb=-aircraft_3.input_max*ones(length_action,1);
        aircraft_3.u_ub=aircraft_3.input_max*ones(length_action,1);
        

        [u_3,fval,exitflag]=quadprog(2*aircraft_3.H,aircraft_3.f,[],[],[],[],aircraft_3.u_lb,aircraft_3.u_ub,[],aircraft_3.options);


%%%%% aircraft 3 for Question 1e

elseif (Index=="1e")

        cvxp = cvx_precision( 'best' );
        cvx_begin  sdp quiet
        
            cvx_solver gurobi
        
            variables u_3(length_action,1)
        
        
            minimize ((aircraft_3.P *u_3 + aircraft_3.Q * aircraft_3.initial_state)' * (aircraft_3.P *u_3 + aircraft_3.Q * aircraft_3.initial_state)+u_3'*u_3 ...,
            + (dual_decom_master.lambda(3,:)-dual_decom_master.lambda(2,:))*(aircraft_3.final_transfer*u_3+aircraft_3.A^5*aircraft_3.initial_state)...,
            + x01'*x01)

            subject to

            sum_square(u_3)<=umax^2;
        
        
        cvx_end
        cvx_precision( cvxp );



%%%%% aircraft 3 for Question 2.1


elseif (Index=="2_1a")

        cvx_clear

        cvx_begin quiet

        variable u_3(length_action,1)

        minimize ((aircraft_3.P *u_3(1:aircraft_4.nx_input) + aircraft_3.Q * aircraft_3.initial_state)' * (aircraft_3.P *u_3(1:aircraft_4.nx_input) + aircraft_3.Q * aircraft_3.initial_state)+u_3'*u_3 ...,
                + (aircraft_3.lambda)*(aircraft_3.final_transfer*u_3+aircraft_3.A^5*aircraft_3.initial_state-ADMM_2a.terminal)...,
                + aircraft_3.rho/2*sum_square(aircraft_3.final_transfer*u_3+aircraft_3.A^5*aircraft_3.initial_state-ADMM_2a.terminal) ...,
                + x03'*x03)


        subject to

                u_3 <= aircraft_3.input_max * ones(length_action,1);
                u_3 >= -aircraft_3.input_max * ones(length_action,1);

        cvx_end

end

% final state
aircraft_3.distributed_final_state=aircraft_3.final_transfer*u_3+aircraft_3.A^5*aircraft_3.initial_state;
aircraft_3.distributed_final_state;

% aircraft_3.history_final_state=[aircraft_3.history_final_state,aircraft_3.distributed_final_state];
aircraft_3.u=u_3;