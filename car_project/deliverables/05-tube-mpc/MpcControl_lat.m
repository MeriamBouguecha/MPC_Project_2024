classdef MpcControl_lat < MpcControlBase
    
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
            x_ref = sdpvar(nx, 1);
            u_ref = sdpvar(nu, 1);

            % Initial states
            x0 = sdpvar(nx, 1);
            x0other = sdpvar(nx, 1); % (Ignore this, not used)

            % Input to apply to the system
            u0 = sdpvar(nu, 1);
            
            %%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
            
            % NOTE: The matrices mpc.A, mpc.B, mpc.C and mpc.D
            %       are the DISCRETE-TIME MODEL of your system
            %       You can find the linearization steady-state
            %       in mpc.xs and mpc.us.
            
            
            % SET THE PROBLEM CONSTRAINTS con AND THE OBJECTIVE obj HERE
            
            X = sdpvar(nx, N);    % (2xN) State trajectory --> [Y position, theta]
            U = sdpvar(nu, N-1);  % (1x(N-1)) Input trajectory --> [delta input]

            % define Q and R matrix
            R = 100;
            Q = diag([0.008, 0.008]);


            % Compute LQR controller for unconstrained system
            [K,Qf,~] = project_dlqr(mpc.A,mpc.B,Q,R);
            % MATLAB defines K as -K, so invert its signal
            K=-K;
            Acl= mpc.A+mpc.B*K;

            % Constraints
            % u in U = { u | Mu <= m }, input
            M = [1;-1]; m = [0.5236; 0.5236];
            % x in X = { x | Fx <= f }, output
            F = [1 0; 0 1; -1 0; 0 -1]; f = [3.5; 0.0873; 0.5; 0.0873];

            % Compute maximal invariant set
            Xf = polytope([F;M*K],[f;m]);
            Acl = [mpc.A+mpc.B*K];
            while 1
                prevXf = Xf;
                [T,t] = double(Xf);
                preXf = polytope(T*Acl,t);
                Xf = intersect(Xf, preXf);
                if isequal(prevXf, Xf)
                    break
                end
            end
            [Ff,ff] = double(Xf);

            % Visualizing the sets
            figure
            hold on; grid on;
            plot(polytope(F,f),'g'); plot(Xf,'r');
            xlabel('Y'); ylabel('Theta');

            obj = 0;
            con = [];

            % initial constraint
            con = [con, X(:,1) == x0];

            for i = 1:N-1

                % state transition equation
                con = [con, X(:,i+1)-mpc.xs == mpc.A * (X(:,i)-mpc.xs) + mpc.B * (U(:,i)-mpc.us)];
                % I have put the delta forumulation, but i don't know if it
                % is the right way

                % constraint on input and output
                con = [con, M * U(:,i) <= m];
                con = [con, F * X(:,i) <= f];

                % objective function
                obj = obj + (X(:,i) - x_ref)' * Q * (X(:,i) - x_ref);
                obj = obj + (U(:,i) - u_ref)' * R * (U(:,i) - u_ref);


            end

            % set final objective
            con = con + (Ff*X(:,N) <= ff);
            obj = obj + (X(:,N) - x_ref)' * Qf * (X(:,N) - x_ref);

            % Replace this line and set u0 to be the input that you
            % want applied to the system. Note that u0 is applied directly
            % to the nonlinear system. You need to take care of any 
            % offsets resulting from the linearization.
            % If you want to use the delta formulation make sure to
            % substract mpc.xs/mpc.us accordingly.

            con = con + ( u0 == U(:,1));
            % not sure if we are in delta formulation or not --> offset?!

            % Pass here YALMIP sdpvars which you want to debug. You can
            % then access them when calling your mpc controller like
            % [u, X, U] = mpc_lat.get_u(x0, ref);
            % with debugVars = {X_var, U_var};
            debugVars = {};
            
            %%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
            
            % Return YALMIP optimizer object
            ctrl_opti = optimizer(con, obj, project_solver(), ...
                {x0, x_ref, u_ref, x0other}, {u0, debugVars{:}});
        end
        
        % Computes the steady state target which is passed to the
        % controller
        function [xs_ref, us_ref] = compute_steady_state_target(mpc, ref)

            %%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
            % INPUTS
            %   ref    - reference to track
            % OUTPUTS
            %   xs_ref, us_ref - steady-state target
            %%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%

            % Steady-state system
            A = mpc.A;
            B = mpc.B;

            %%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
            xs_ref = [ref;0];
            us_ref = 0;
            %%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
        end
    end
end
