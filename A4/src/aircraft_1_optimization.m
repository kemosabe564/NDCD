%%%%%  aircraft 1

%%%%% aircraft 1 for Question 1a & 1b


if (Index == "1a" || Index == "1b")

        aircraft_1.options=optimoptions('quadprog', 'Display', 'off');

        aircraft_1.H = aircraft_1.P'*aircraft_1.P;
        aircraft_1.H = aircraft_1.H+eye(length_action);
        aircraft_1.f = aircraft_1.initial_state'*aircraft_1.Q'*aircraft_1.P*2;
        aircraft_1.f = aircraft_1.f+(decompose_problem.lambda(1,:)-decompose_problem.lambda(4,:))*aircraft_1.matrix;
        aircraft_1.f = aircraft_1.f';

        aircraft_1.u_lb=-aircraft_1.input_max*ones(length_action,1);
        aircraft_1.u_ub=aircraft_1.input_max*ones(length_action,1);
        

        [u1,fval,exitflag]=quadprog(2*aircraft_1.H,aircraft_1.f,[],[],[],[],aircraft_1.u_lb,aircraft_1.u_ub,[],aircraft_1.options);


%%%%% aircraft 1 for Question 1c


elseif (Index=="1c" )

        aircraft_1.options=optimoptions('quadprog','Display','off');

        aircraft_1.H=aircraft_1.P'*aircraft_1.P;
        aircraft_1.H=aircraft_1.H+eye(length_action);
        aircraft_1.f=aircraft_1.initial_state'*aircraft_1.Q'*aircraft_1.P*2;
        aircraft_1.f=aircraft_1.f+(Nesterov.lambda(1,:)-Nesterov.lambda(4,:))*aircraft_1.matrix;
        aircraft_1.f=aircraft_1.f';

        aircraft_1.u_lb=-aircraft_1.input_max*ones(length_action,1);
        aircraft_1.u_ub=aircraft_1.input_max*ones(length_action,1);
        

        [u1,fval,exitflag]=quadprog(2*aircraft_1.H,aircraft_1.f,[],[],[],[],aircraft_1.u_lb,aircraft_1.u_ub,[],aircraft_1.options);


%%%%% aircraft 1 for Question 1d


elseif (Index=="1d")

        aircraft_1.options=optimoptions('quadprog','Display','off');

        aircraft_1.H=aircraft_1.P'*aircraft_1.P;
        aircraft_1.H=aircraft_1.H+eye(length_action);
        aircraft_1.f=aircraft_1.initial_state'*aircraft_1.Q'*aircraft_1.P*2;
        aircraft_1.f=aircraft_1.f+(aircraft_1.lambda(1,:)-aircraft_1.lambda(4,:))*aircraft_1.matrix;
        aircraft_1.f=aircraft_1.f';

        aircraft_1.u_lb=-aircraft_1.input_max*ones(length_action,1);
        aircraft_1.u_ub=aircraft_1.input_max*ones(length_action,1);

        [u1,fval,exitflag]=quadprog(2*aircraft_1.H,aircraft_1.f,[],[],[],[],aircraft_1.u_lb,aircraft_1.u_ub,[],aircraft_1.options);
        

%%%%% aircraft 1 for Question 1e


elseif (Index=="1e")

        cvxp = cvx_precision( 'best' );
        cvx_begin  sdp quiet
        
            cvx_solver gurobi
        
            variables u1(length_action,1)
        
            minimize ((aircraft_1.P *u1 + aircraft_1.Q * aircraft_1.initial_state)' * (aircraft_1.P *u1 + aircraft_1.Q * aircraft_1.initial_state)+u1'*u1 ...,
            + (decompose_problem.lambda(1,:)-decompose_problem.lambda(4,:))*(aircraft_1.matrix*u1+aircraft_1.A^5*aircraft_1.initial_state)...,
            + x01'*x01)

            subject to

            sum_square(u1)<=umax^2;

        
        cvx_end
        cvx_precision( cvxp );


%%%%% aircraft 1 for Question 2.1


elseif (Index=="2_1a")

        cvx_clear

        cvx_begin quiet

        variable u1(length_action,1)

        minimize ((aircraft_1.P *u1(1:aircraft_4.nx_input) + aircraft_1.Q * aircraft_1.initial_state)' * (aircraft_1.P *u1(1:aircraft_4.nx_input) + aircraft_1.Q * aircraft_1.initial_state)+u1'*u1 ...,
                + (aircraft_1.lambda)*(aircraft_1.matrix*u1+aircraft_1.A^5*aircraft_1.initial_state-ADMM_2a.terminal)...,
                + aircraft_1.rho/2*sum_square(aircraft_1.matrix*u1+aircraft_1.A^5*aircraft_1.initial_state-ADMM_2a.terminal))


        subject to

                u1 <= aircraft_1.input_max * ones(length_action,1);
                u1 >= -aircraft_1.input_max * ones(length_action,1);

        cvx_end

end


% final_state of aircraft 1

aircraft_1.distributed_final_state=aircraft_1.matrix*u1+aircraft_1.A^5*aircraft_1.initial_state;
aircraft_1.distributed_final_state;


% aircraft_1.history_final_state=[aircraft_1.history_final_state,aircraft_1.distributed_final_state]

aircraft_1.u=u1;


