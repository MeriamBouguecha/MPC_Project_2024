classdef MpcControl_lon < MpcControlBase
    
    methods
        % Design a YALMIP optimizer object that takes a steady-state state
        % and input (xs, us) and returns a control input
        function ctrl_opti = setup_controller(mpc)
            
            %%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
            % INPUTS
            %   x0           - initial state (estimate)
            %   x_ref, u_ref - reference state/input
            % OUTPUTS
            %   u0           - input to apply to the system
            %%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
            
            N_segs = ceil(mpc.H/mpc.Ts); % Horizon steps
            N = N_segs + 1;              % Last index in 1-based Matlab indexing

            [nx, nu] = size(mpc.B);
            
            % Targets
            V_ref = sdpvar(1);
            u_ref = sdpvar(1);

            % Disturbance estimate (Ignore this before Todo 4.1)
            d_est = sdpvar(1);

            % Initial states
            x0 = sdpvar(nx, 1);
            x0other = sdpvar(nx, 1); %Lead car initial state

            % Input to apply to the system
            u0 = sdpvar(nu, 1);
            
            %%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
            
            % NOTE: The matrices mpc.A, mpc.B, mpc.C and mpc.D
            %       are the DISCRETE-TIME MODEL of your system
            %       You can find the linearization steady-state
            %       in mpc.xs and mpc.us.
 
            load('tube_mpc_data.mat', 'X_t', 'U_t', 'X_f', 'Q', 'K', 'R', 'P', 'Qf', 'x_safe'); 

            
            %Delta Dynamics:
            Delta_x = sdpvar(nx, N, 'full'); % Relative state trajectory
            U = sdpvar(nu, N-1, 'full');%Input
            Z = sdpvar(nx, N, 'full'); %Nominal X 
            V = sdpvar(nu, N-1, 'full');%Nominal input
            E = sdpvar(nx, N, 'full'); %Error Dynamics 
            
            % Extract tightened constraint matrices
            F_tight = X_t.A;
            M_tight = U_t.A;
            f_tight = X_t.b;
            m_tight = U_t.b;

            % Constraints and cost function
            con = [];
            obj = 0;
            
            % Initialize relative state
            % Initialize relative state
            state_init = [x0other(1)- x0(1) - x_safe ; % Relative position (ego - lead
                          x0other(2)- x0(2)]; % Relative velocity (ego - lead)
                     
            con = [con, Delta_x(:, 1) == state_init];
            %Same used for terminal components 

            for i = 1:N-1
                %con = [ con, delta(:, i+1) == mpc.A* (delta(:, i)) - mpc.B*(u(:,i))  ];  
                con = [con, Z(:, i+1) == mpc.A* (Z(:, i)) - mpc.B*(V(:,i))];  
                con = [con, E(:, i+1) == (mpc.A- mpc.B*K)*E(:, i) ]; %i'm very very unsure of this
                con = [con, U(:,i) == K*(Delta_x(:,i)-Z(:,i)) + V(:,i)];
             
                con = [con, M_tight*(V(:,i)) <= m_tight];
                con = [con, F_tight*(Z(:,i)) <= f_tight];
                
                obj = obj + Z(:,i)'*P*Z(:,i) + (V(:,i))'*R*(V(:,i));
            end

            con = [con, X_f.A*(Z(:,N)) <= X_f.b];
            obj = obj + Z(:,N)'*Qf*Z(:,N);

            % Control input applied at the first step
            %con = con + ( u0 == (K * (Delta_x(:,1) - Z(:, 1)) +V(:, 1) + mpc.us));
            con = [con, u0 == U(:,1)];

            % Pass here YALMIP sdpvars which you want to debug. You can
            % then access them when calling your mpc controller like
            % [u, X, U] = mpc_lat.get_u(x0, ref);
            % with debugVars = {X_var, U_var};
            debugVars = {}; %Z,V,E,U,Delta_x
            
            %%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
            
            % Return YALMIP optimizer object
            ctrl_opti = optimizer(con, obj, project_solver(), ...
                {x0, V_ref, u_ref, d_est, x0other}, {u0, debugVars{:}});
        end
        
        % Computes the steady state target which is passed to the
        % controller
        function [xs_ref, us_ref] = compute_steady_state_target(mpc, ref, d_est)

            %%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
            % INPUTS
            %   ref    - reference to track
            % OUTPUTS
            %   xs_ref, us_ref - steady-state target
            %%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%

            % Steady-state system
            A = mpc.A(2, 2);
            B = mpc.B(2, 1);

            % Subsystem linearization steady-state
            xs = mpc.xs(2);
            us = mpc.us;

            %%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
            xs_ref = ref; 
            us_ref = (ref - xs - A*(ref - xs))/B + us;  

            %%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
        end
    end
end
