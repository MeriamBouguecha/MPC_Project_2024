clear all;
close all;

%% System initialisation

Ts = 1/10; % Sample time
car = Car(Ts);
[xs, us] = car.steady_state(120 / 3.6); 
sys = car.linearize(xs, us);
[sys_lon, sys_lat] = car.decompose(sys);
H_lon = 20; % Horizon length in seconds

mpc_lon = MpcControl_lon(sys_lon, Ts, H_lon);
mpc_lat = MpcControl_lat(sys_lat, Ts, H_lon);
mpc = car.merge_lin_controllers(mpc_lon, mpc_lat);


%% TEST 1 : 
% 
% % Control input :
% otherRef = 100 / 3.6; 
% 
% % Simulation :
% params = struct();
% params.Tf = 25;
% params.myCar.model = car;
% params.myCar.x0 = [0 0 0 100/3.6]'; 
% params.myCar.u = @mpc.get_u; 
% params.myCar.ref = [0 120/3.6]';
% params.otherCar.model = car;
% params.otherCar.x0 = [15 0 0 otherRef]';
% params.otherCar.u = car.u_const(100/3.6); 
% result = simulate(params); 
% visualization(car, result);
% 

%% TEST 2 : 

% Simulation :
params = struct();
params.Tf = 25;
params.myCar.model = car;
params.myCar.x0 = [0 0 0 115/3.6]'; 
params.myCar.u = @mpc.get_u; 
params.myCar.ref = [0 120/3.6]';
params.otherCar.model = car;
params.otherCar.x0 = [8 0 0 120/3.6]';
params.otherCar.u = car.u_fwd_ref(); 
params.otherCar.ref = car.ref_robust();
result = simulate(params); 
visualization(car, result); 
