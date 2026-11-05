% Ts = 0.1; 
% car = Car(Ts); 
% [xs, us] = car.steady_state(120/3.6);
% sys =car.linearize(xs,us);
% [sys_lon, sys_lat] = car.decompose(sys); 
% 
% H_lon = 1; 
% mpc_lon = MpcControl_lon(sys_lon, Ts, H_lon);
clc; clear; close all;
%%
% Set simulation parameters
Ts = 1/10; % Sample time
H_lon = 5; % Horizon for longitudinal controller
H_lat = 5; % Horizon for lateral controller

% Initialize the car model
car = Car(Ts);

% Find steady-state operating point for 120 km/h
[xs, us] = car.steady_state(120 / 3.6);

% Linearize system around operating point
sys = car.linearize(xs, us);

% Decompose into longitudinal and lateral subsystems
[sys_lon, sys_lat] = car.decompose(sys);

% Design MPC controllers
mpc.lon = MpcControl_lon(sys_lon, Ts, H_lon);
mpc.lat = MpcControl_lat(sys_lat, Ts, H_lat);

% Merge controllers
mpc.car = car.merge_lin_controllers(mpc.lon, mpc.lat);

% Define initial conditions and reference
x0 = [0; 0; 0; 80 / 3.6]; % [x; y; theta; V]
ref1 = [0; 80 / 3.6];              % Initial reference [y_ref; V_ref]
ref2 = [3; 120 / 3.6];             % Change reference after 5 seconds

% Simulation parameters
params = struct();
params.Tf = 15; % Simulation duration in seconds
params.myCar.model = car;
params.myCar.x0 = x0;
params.myCar.u = mpc.car.get_u;
params.myCar.ref = car.ref_step(ref1, ref2, 5); % Reference step at 5s
% Run simulation
result = simulate(params);
% Visualize results
visualization(car, result);
