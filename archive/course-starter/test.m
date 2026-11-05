% Ts = 0.1;
% car = Car(Ts); 
% 
% delta = deg2rad(0);
% u_T = 0; 
% 
% x_pos = 0; 
% y_pos = 0; 
% theta = 0; 
% V = 5; 
% 
% u = [delta, u_T]'; 
% x = [x_pos, y_pos, theta, V]'; 
% x_dot = car.f(x, u); 

car = Car(Ts);
Tf = 4.0; % endtime of simulation 

x0 = [0, 0, deg2rad(0), 30/3.6]'; % start state
u = [deg2rad(-1), -0.1]'; % (delta, uT) --> constant input

% setup simulation struct
params = {};
params.Tf = Tf;
params.myCar.model = car;
params.myCar.x0 = x0;
params.myCar.u = u;

% Simulate nonlinear model
result = simulate(params);
visualization(car, result);