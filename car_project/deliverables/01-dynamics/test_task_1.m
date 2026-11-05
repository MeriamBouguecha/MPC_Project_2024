% Ts = 1/10; 
% car = Car(Ts); 
% 
% %define values for input u and state x
% delta = 0; % steering angle
% u_T = 0; % throttle
% 
% x = 0; % position 
% y = 0; % position
% theta = 0; % position
% V = 20; % velocity
% 
% u = [delta, u_T]';
% x = [x, y, theta, V]'; 
% x_dot = car.f(x, u); 

%1.2
Ts = 0.1;
car = Car(Ts); 
Tf = 2.0; 

x0 = [0, 0, deg2rad(-2), 20/3.6]'; 
u = [deg2rad(1); 0.7]; 

params = struct(); 
params.Tf = Tf; 
params.myCar.model = car; 
params.myCar.x0 = x0; 
params.myCar.u = u; 
result = simulate(params); 
visualization(car, result); 

