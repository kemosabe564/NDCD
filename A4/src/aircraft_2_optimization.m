%%%%%  aircraft 2

%%%%% aircraft 2 for Question 1a

if (Index=="1a" || Index=="1b")

        aircraft_2.options=optimoptions('quadprog','Display','off');

        aircraft_2.H=aircraft_2.P'*aircraft_2.P;
        aircraft_2.H=aircraft_2.H+eye(length_action);
        aircraft_2.f=aircraft_2.initial_state'*aircraft_2.Q'*aircraft_2.P*2;
        aircraft_2.f=aircraft_2.f+(decompose_problem.lambda(2,:)-decompose_problem.lambda(1,:))*aircraft_2.matrix;
        aircraft_2.f=aircraft_2.f';

        aircraft_2.u_lb=-aircraft_2.input_max*ones(length_action,1);
        aircraft_2.u_ub=aircraft_2.input_max*ones(length_action,1);
        

        [u2,fval,exitflag]=quadprog(2*aircraft_2.H,aircraft_2.f,[],[],[],[],aircraft_2.u_lb,aircraft_2.u_ub,[],aircraft_2.options);

%%%%% aircraft 2 for Question 1c


elseif (Index=="1c" )

        aircraft_2.options=optimoptions('quadprog','Display','off');

        aircraft_2.H=aircraft_2.P'*aircraft_2.P;
        aircraft_2.H=aircraft_2.H+eye(length_action);
        aircraft_2.f=aircraft_2.initial_state'*aircraft_2.Q'*aircraft_2.P*2;
        aircraft_2.f=aircraft_2.f+(Nesterov.lambda(2,:)-Nesterov.lambda(1,:))*aircraft_2.matrix;
        aircraft_2.f=aircraft_2.f';

        aircraft_2.u_lb=-aircraft_2.input_max*ones(length_action,1);
        aircraft_2.u_ub=aircraft_2.input_max*ones(length_action,1);
        

        [u2,fval,exitflag]=quadprog(2*aircraft_2.H,aircraft_2.f,[],[],[],[],aircraft_2.u_lb,aircraft_2.u_ub,[],aircraft_2.options);


%%%%%% aircraft 2 for Question 1d %%%%%

elseif (Index=="1d")

        aircraft_2.options=optimoptions('quadprog','Display','off');

        aircraft_2.H=aircraft_2.P'*aircraft_2.P;
        aircraft_2.H=aircraft_2.H+eye(length_action);
        aircraft_2.f=aircraft_2.initial_state'*aircraft_2.Q'*aircraft_2.P*2;
        aircraft_2.f=aircraft_2.f+(aircraft_2.lambda(2,:)-aircraft_2.lambda(1,:))*aircraft_2.matrix;
        aircraft_2.f=aircraft_2.f';

        aircraft_2.u_lb=-aircraft_2.input_max*ones(length_action,1);
        aircraft_2.u_ub=aircraft_2.input_max*ones(length_action,1);
        

        [u2,fval,exitflag]=quadprog(2*aircraft_2.H,aircraft_2.f,[],[],[],[],aircraft_2.u_lb,aircraft_2.u_ub,[],aircraft_2.options);



%%%%% aircraft 2 for Question 1e


elseif (Index=="1e")

        cvxp = cvx_precision( 'best' );
        cvx_begin  sdp quiet
        
            cvx_solver gurobi
        
            variables u2(length_action,1)
        
            minimize ((aircraft_2.P *u2 + aircraft_2.Q * aircraft_2.initial_state)' * (aircraft_2.P *u2 + aircraft_2.Q * aircraft_2.initial_state)+u2'*u2 ...,
            + (decompose_problem.lambda(2,:)-decompose_problem.lambda(1,:))*(aircraft_2.matrix*u2+aircraft_2.A^5*aircraft_2.initial_state)...,
            + x01'*x01)

            subject to

            sum_square(u2)<=umax^2;
        
        cvx_end
        cvx_precision( cvxp );


        
%%%%% aircraft 2 for Question 2.1


elseif (Index=="2_1a")

        cvx_clear

        cvx_begin quiet

        variable u2(length_action,1)

        minimize ((aircraft_2.P *u2(1:aircraft_4.nx_input) + aircraft_2.Q * aircraft_2.initial_state)' * (aircraft_2.P *u2(1:aircraft_4.nx_input) + aircraft_2.Q * aircraft_2.initial_state)+u2'*u2 ...,
                + (aircraft_2.lambda)*(aircraft_2.matrix*u2+aircraft_2.A^5*aircraft_2.initial_state-ADMM_2a.terminal)...,
                + aircraft_2.rho/2*sum_square(aircraft_2.matrix*u2+aircraft_2.A^5*aircraft_2.initial_state-ADMM_2a.terminal)...,
                + x02'*x02)


        subject to

                u2 <= aircraft_2.input_max * ones(length_action,1);
                u2 >= -aircraft_2.input_max * ones(length_action,1);

        cvx_end

end

% final state

aircraft_2.distributed_final_state=aircraft_2.matrix*u2+aircraft_2.A^5*aircraft_2.initial_state;
aircraft_2.distributed_final_state;

aircraft_2.u=u2;