clc; clear; close all;
%% "offline" part:
Ts = 1/10; % Sample time (10 Hz)
car = Car(Ts);
[xs, us] = car.steady_state(100 / 3.6);
sys = car.linearize(xs, us);
[sys_lon, sys_lat] = car.decompose(sys); 
sys_lon_d = c2d(sys_lon, Ts); % Discretize 
[A, B, C, D] = ssdata(sys_lon_d); % Extract discrete-time matrices

%% lead car

% disturbance bounds for the lead car
M = [1;-1]; m = [us(2)+0.5; -us(2)+0.5];

% calculate Polyhedron for the disturbance
W = Polyhedron(M,m);  
W_state = B*W ; 

%% LQR

% Define LQR cost matrices
Q = diag([50, 1]);  % difference
R = 0.1;


% Compute LQR feedback gain
[K, Qf, ~] = dlqr(A, -B, Q, R); % good
K = -K;
A_cl = A - B * K;

% Check stabilization of K
eigenvalues = eig(A_cl);
if all(abs(eigenvalues) < 1)
    disp('The closed-loop system is stable.');
    disp(eigenvalues);
    disp('K') ; 
    disp(K) ; 
else
    error('The closed-loop system is NOT stable. Adjust Q and R.');
end

%% Calculate minimal invariant set
% Compute E
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
plot(E)
legend('E : Minimum robust invariant set')
xlabel('Delta X')
ylabel('Delta V')
grid on


%% Constraints tightening

% x safe
x_safe = 10 ; 

% input constraints
M = [1;-1]; m = [1; 1];

% x in X = { x | Fx <= f } with the safety
F = [-1 0]; 
f = -6 + x_safe; % x_ego - x_lead - x_safe > 6 - x_safe  

X = Polyhedron('A',F,'b',f);
U = Polyhedron('A',M,'b',m);

%tightened constraints 
X_t = X - E;
U_t = U - K*E;

%P matrix
P = dlyap(A_cl',Q + K'*R*K);

% Check if terminal set contains the origin
if ~X_t.contains([0; 0])
    fprintf('Tightened X set does not contain the origin.'); %for feasibility  
end 


%% Terminal set
F_t = X_t.A;
M_t = U_t.A;
f_t = X_t.b;
m_t = U_t.b;

X_f = Polyhedron([F_t;M_t*K],[f_t;m_t]);

max_iter = 100;
for i = 1:max_iter
    prevXf = X_f;
    T_Xf = X_f.A;
    t_Xf = X_f.b;
    X_f_new = Polyhedron(T_Xf*A_cl,t_Xf);
    X_f = intersect(X_f, X_f_new); % Intersection of sets
    if abs(X_f.volume - prevXf.volume) < 1e-10
        fprintf("Get our terminal set after %i iterations\n", i)
        break
    end
    fprintf("iteration %i to get our Terminal set\n",i);
end 

% Plot for X_f
figure
plot(X_f)
legend('Xf : Terminal Set')
xlabel('Delta X')
ylabel('Delta V')
grid on


%% Save Data
save('tube_mpc_data.mat', 'X_t', 'U_t', 'X_f', 'Q', 'K', 'R', 'P', 'Qf', 'x_safe');
