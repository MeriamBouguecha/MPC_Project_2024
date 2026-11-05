clc; clear; close all;
%% "offline" part: 
Ts = 1/10; % Sample time (10 Hz)
H_lon = 20; % Horizon for longitudinal controller
car = Car(Ts);
[xs, us] = car.steady_state(100 / 3.6);
sys = car.linearize(xs, us);
[sys_lon, sys_lat] = car.decompose(sys); 
sys_lon_d = c2d(sys_lon, Ts); % Discretize 
[A, B, C, D] = ssdata(sys_lon_d); % Extract discrete-time matrices

M = [1;-1]; m = [us(2)+0.5; -us(2)+0.5]; %disturbance bounds 
W = Polyhedron( M, m);  

W_state = B*W ; 

% Define LQR cost matrices
Q =1*diag([50, 1]); 
R = 0.1;

% Compute LQR feedback gain
[K, ~, ~] = dlqr(A, -B, Q, R);
K = -K;

% Check stabilization of K
A_cl = A - B * K;
eigenvalues = eig(A_cl);
if all(abs(eigenvalues) < 1)
    disp('The closed-loop system is stable.');
    disp(eigenvalues);
    disp('K') ; 
    disp(K) ; 
else
    error('The closed-loop system is NOT stable. Adjust Q and R.');
end

E = Polyhedron.emptySet(2); %initialization
max_iterations = 100;
i = 1;
while true
    E_prev = E ; 
    E = E_prev + A_cl^i*W_state; % if E and W are Polyhedron, the operator + acts as Minkowski sum and not normal addition in MPT
    E = minHRep(E);
    if norm(A_cl^i) < 1e-2   %check if diff is sufficiently small to terminate
        fprintf('Converged after %d iterations (norm-based stopping).\n', i);
            break;
    end
    i = i+1;
    fprintf('Iteration %d: Not yet converged.\n', i);
end
    
if i == max_iterations
    fprintf("E didn't pass vibe check")
end

figure
plot([W_state E])
legend('Mapped Disturbance Set W_{state}', 'Minimal Robust Invariant Set \epsilon')
xlabel('\Delta x (Position Deviation [m])')
ylabel('\Delta v (Velocity Deviation [m/s])')
grid on

%% Constraints tightening :'( 

%Constraints: 
x_safe = 7 ; 
M = [1;-1]; m = [1; 1]; %input constraints
% x in X = { x | Fx <= f }Safety constraint
F = [-1 0]; 
f = [ (-6 + x_safe) ];% x_ego - x_lead - x_safe > 6 - x_safe  

X = Polyhedron('A',F,'b',f);
U = Polyhedron('A',M,'b',m);

%tightened constraints 
X_t = X - E;
U_t = U - K*E;

% Check if terminal set contains the origin
if ~X_t.contains([0; 0])
    fprintf('Tightened X set does not contain the origin.'); %for feasibility  
end 

% Improved minimal plot
% Improved minimal plot with visual clarity
figure;
hold on;
plot(X, 'color', 'red', 'LineWidth', 1.5); % Original set X in red
plot(X_t, 'color', 'blue', 'LineWidth', 0.5); % Tightened set X_t in blue
legend("X (Original Constraints)", "X_t (Tightened Constraints)", 'Location', 'northeast');
xlabel('Position Deviation (\Delta x) [m]');
ylabel('Velocity Deviation (\Delta v) [m/s]');
title('Comparison of State Constraints (X vs. X_t)');
grid on;



figure 
plot([U U_t])
legend("U", "tightened U")
xlabel('Throttle Input \( u_T \)')
ylabel('N/A (1D input space)')
grid on



%%
% MPC LQR parameters for terminal components 
Q = 1*diag([60, 60]); % State cost weights
R = 0.01;            % Input cost weight

% Compute terminal feedback gain , Q and R of the mpc controller!! not the
% same as the begining as in the example in the class 

[K1, P, ~] = dlqr(A, -B, Q, R); % Compute feedback gain
P = dlyap(A_cl',Q + K'*R*K);

% Check stability of the closed-loop system
A_cl1 = A - B * K1; % Closed-loop dynamics
if all(abs(eig(A_cl1)) < 1)
    disp('The closed-loop system is stable.');
    disp(eig(A_cl1)); 
else
    fprintf('The closed-loop system is NOT stable.\n');
    disp(eig(A_cl1)); 
end % Closing the "if" statement
%%
% Compute terminal set
X_f = X_t; % Start with tightened state constraints
max_iter = 100;
for i = 1:max_iter
    X_f_new = X_t & (A_cl1 * X_f); % Intersection of sets
    if X_f_new == X_f
        disp('Converged to terminal set.');
        break;
    end
    X_f = X_f_new;
end 



% Terminal weight P is already computed via dlqr
disp('Terminal components computed successfully.')

figure
plot(X_f)
legend('X_f (Terminal Set)')
xlabel('Relative Position (\Delta x, meters)') % x-axis is the relative position
ylabel('Relative Velocity (\Delta v, meters/second)') % y-axis is the relative velocity
grid on
%%
save('tube_mpc_data.mat', 'X_t', 'U_t', 'X_f', 'P');
%%
% Map disturbance set W to state space: W_state = W * B
%W_vertices = W.V; % Extract vertices of W
%W_mapped_vertices = (-B * W_vertices')'; % Map vertices through B
%W_state = Polyhedron('V', W_mapped_vertices); % Create new polyhedron


% % Initialize invariant set with the mapped disturbance set
% E = W_state;
% 
% % Iterative computation of the invariant set
% max_iter = 100;
% tolerance = 1e-2;
% A_cl_i = eye(size(A_cl)); % Initialize A_cl^i as identity matrix
% Ai_W = W_state;  
% i = 1;
% 
% figure(1); clf; % Visualization of iterations
% hold on;
% 
% %plot(E, 'color', 'b'); % Specify the color as a key-value pair
% 
% while i <= max_iter
%     prevE = E;
%     A_cl_i = A_cl_i * A_cl;
%     Ai_W = A_cl_i * W_state + Ai_W;
%     Z = zeros(size(Ai_W.A, 1), size(prevE.A, 2)); 
%     %E = projection(polytope([prevE.A -prevE.A ; Z Ai_W.A] , [prevE.b ; Ai_W.b]) , A:size(prevE.A,2)); 
%     %Minkowski Sum
%     NewSet = Polyhedron(...
%         'A', [prevE.A, -prevE.A; Z, Ai_W.A], ...
%         'b', [prevE.b; Ai_W.b] ...
%     );
%     E = projection(NewSet, 1:size(prevE.A, 2));
%     % Plot intermediate sets
%     %plot(E, 'color', 'y');
%     %plot(Ai_W, 'color', 'r');
%     %plot(prevE, 'color', 'g');
% 
%     if norm(A_cl_i, 'inf') < tolerance
%         fprintf('Converged after %d iterations (norm-based stopping).\n', i);
%         break;
%     end
% 
%     %fprintf('Iteration %d: Not yet converged.\n', i);
%     %pause(0.5);
%     i = i + 1;
% end
% 
% if i > max_iter
%     warning('Maximum iterations reached before convergence.');
% end
% 
% % Final visualization
% plot(E, 'color', 'k'); % Final invariant set
% legend({'Initial Set (W)', 'Intermediate Sets (yellow)', 'Accumulated Ai_W (red)', 'Previous Set (green)', 'Final Invariant Set (black)'});
% title('Computation of Minimum Robust Invariant Set');
% hold off;

% Save results
%sets.K = K;
%sets.E = E;
%save(filename, 'sets');
