% 
Ts = 10 ; 
car = Car(1/Ts) ; 
H =10 ; 
nmpc = NmpcControl(car,H) ; 

x0 = [0 0 0 80/3.6]' ;
ref = [3 100/3.6]' ;
u = nmpc.get_u(x0, ref) ; %check if open-loop predictions are reasonable 


params = struct();
params.Tf = 15;
params.myCar.model = car;
params.myCar.x0 = [0 0 0 80/3.6]';
params.myCar.u = @nmpc.get_u;
ref1 = [0 80/3.6]';
ref2 = [3 100/3.6]';
params.myCar.ref = car.ref_step(ref1, ref2, 2);
result = simulate(params);
visualization(car, result);

