classdef LonEstimator
    properties
        % continous sub-system
        sys
        % Extended linearization points
        xs_hat, us_hat
        % Extended system matrices
        A_hat, B_hat, C_hat
        % Observer gain matrix
        L
    end
    
    methods
        % Setup the longitudinal disturbance estimator
        function est = LonEstimator(sys, Ts)

            xs = sys.UserData.xs;
            us = sys.UserData.us;
            
            % Discretize the system and extract the A,B,C,D matrices
            [~, Ad, Bd, Cd, ~] = Car.c2d_with_offset(sys, Ts);

            %%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%

            Ad_red = Ad(2,2);
            Bd_red = Bd(2);
            Cd_red = Cd(2,2);

            % Extended system matrices (adjusting sizes)
            A_hat = [Ad_red(1), Bd_red(1); zeros(1, size(Ad_red, 1)), 1];
            B_hat = [Bd_red(1); 0];
            C_hat = [Cd_red(1), 0];

            est.A_hat = A_hat;
            est.B_hat = B_hat;
            est.C_hat = C_hat;

            disp('A_hat')
            disp(A_hat)
            disp('B_hat')
            disp(B_hat)
            disp('C_hat')
            disp(C_hat)

            % Exact two-state observer pole placement; same original poles.
            p = [0.5,0.6]; a = Ad_red; b = Bd_red;
            l1 = a+1-sum(p);
            L = -[l1; (prod(p)-a+l1)/b];
          
            disp('Gain L :');
            disp(L);

            % Extended state dynamics
            est.xs_hat = [xs(2); zeros(1, 1)]; % Include disturbance states
            est.us_hat = us;

            est.L = L;

            disp('xs_hat');
            disp(est.xs_hat)
            disp('us_hat');
            disp(est.us_hat)
            
            %%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
        end
        
        % This function takes in the the estimate, input, and measurement
        % at timestep i and predicts the next (extended) state estimate at
        % the next timestep i + 1.
        function z_hat_next = estimate(est, z_hat, u, y)
            %%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
            % INPUTS
            %   z_hat      - current estimate (V, dist)
            %   u          - longitudinal input (u_T)
            %   y          - longitudinal measurement (x, V)
            % OUTPUTS
            %   z_hat_next - next time step estimate
            %%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%

            %%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%

            % Estimation equation
            z_hat_next = est.xs_hat+ est.A_hat*(z_hat-est.xs_hat)+ ...
            est.B_hat*(u - est.us_hat)+est.L*(est.C_hat*z_hat-y);

            %disp('y');
            %disp(y);
            %disp('est.C_hat*z_hat')
            %disp(est.C_hat*z_hat)

            %%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
        end
    end
end
