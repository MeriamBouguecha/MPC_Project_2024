function result = run_project(part,showPlots)
% RUN_PROJECT Run the recovered team implementation in a selected scenario.
% addpath('car_project'); result = run_project('nonlinear-tracking');
% External toolboxes must already be on the MATLAB path (see README).
if nargin < 1, part = 'nonlinear-tracking'; end
if nargin < 2, showPlots = true; end
setup_project(part);
car = Car(0.1);
params = struct('Tf',15);
params.myCar = struct('model',car,'x0',[0;0;0;80/3.6]);
switch part
    case 'dynamics'
        params.Tf = 2;
        params.myCar.x0 = [0;0;deg2rad(-2);20/3.6];
        params.myCar.u = [deg2rad(1);0.7];
    case 'linearization'
        [xs,us] = car.steady_state(120/3.6);
        result = car.linearize(xs,us);
        disp(result); return;
    case {'tracking','offset-free'}
        [xs,us] = car.steady_state(120/3.6);
        [lon,lat] = car.decompose(car.linearize(xs,us));
        mpcLon = MpcControl_lon(lon,car.Ts,5);
        mpcLat = MpcControl_lat(lat,car.Ts,5);
        controller = car.merge_lin_controllers(mpcLon,mpcLat);
        params.myCar.u = controller.get_u;
        if strcmp(part,'offset-free')
            est = LonEstimator(lon,car.Ts);
            params.myCar.est_fcn = @est.estimate;
            params.myCar.est_dist0 = 0;
            params.myCar.ref = car.ref_step([0;80/3.6],[3;50/3.6],2);
        else
            params.myCar.ref = car.ref_step([0;80/3.6],[3;120/3.6],5);
        end
    case {'nonlinear-tracking','nonlinear-avoidance'}
        % Preserve the team's 10 s horizon and original objective weights.
        controller = NmpcControl(car,10);
        params.myCar.u = @controller.get_u;
        if strcmp(part,'nonlinear-tracking')
            params.myCar.ref = car.ref_step([0;80/3.6],[3;100/3.6],2);
        else
            params.myCar.ref = car.ref_step([0;80/3.6],[0;100/3.6],1);
            params.otherCar = struct('model',car,'x0',[20;0;0;80/3.6], ...
                'u',car.u_const(80/3.6));
        end
    case 'tube-mpc'
        error('Project:TubeReview', ['The recovered tube MPC sources are preserved, but have unresolved ' ...
            'model/set inconsistencies. See docs/VALIDATION.md before running the legacy script.']);
end
result = simulate(params);
assert(result.T(end) >= params.Tf,'Project:Aborted','Simulation aborted before its requested end time.');
if showPlots, visualization(car,result); end
end
