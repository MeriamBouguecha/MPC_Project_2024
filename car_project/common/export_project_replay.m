function all = export_project_replay(results,names)
% Adapt original team's result structs to the offline telemetry replay.
here = fileparts(mfilename('fullpath')); root = fileparts(fileparts(here));
dataDir = fullfile(root,'docs','data');
if ~exist(dataDir,'dir'), mkdir(dataDir); end
all = cell(size(results));
for i = 1:numel(results)
    r = results{i}; name = names{i};
    switch name
        case 'nonlinear-tracking'
            title = 'Nonlinear MPC lane change';
            description = 'Recovered team controller: a 3 m lane change, from 80 to 100 km/h.';
        case 'nonlinear-avoidance'
            title = 'Nonlinear MPC with a lead car';
            description = 'Recovered team controller: elliptical separation from a slower lead vehicle.';
        case 'tracking'
            title = 'Linear MPC tracking';
            description = 'Recovered team controller: lane change and acceleration from 80 to 120 km/h.';
        case 'offset-free'
            title = 'Offset-free MPC tracking';
            description = 'Recovered team controller and observer: deceleration from 80 to 50 km/h.';
    end
    x = r.myCar.X; u = r.myCar.U; ref = r.myCar.Ref;
    s = struct('id',name,'title',title,'description',description, ...
        'dt',r.T(2)-r.T(1),'t',r.T,'states',x','inputs',u(:,1:end-1)', ...
        'reference',ref','disturbance',zeros(size(r.T)), ...
        'bias',zeros(size(r.T)),'predictionY',zeros(0,30));
    if isfield(r.myCar,'Z_hat'), s.disturbance = r.myCar.Z_hat(end,:); end
    if isfield(r,'otherCar'), s.otherStates = r.otherCar.X'; else, s.otherStates = []; end
    s.source = 'Recovered team implementation · niklashcs/mpc_epfl';
    m = struct();
    m.finalLateralError = abs(x(2,end)-ref(1,end));
    m.finalSpeedErrorKmh = abs(x(4,end)-ref(2,end))*3.6;
    m.maxHeadingDeg = max(abs(rad2deg(x(3,:))));
    m.maxSteeringDeg = max(abs(rad2deg(u(1,:))));
    m.minY = min(x(2,:)); m.maxY = max(x(2,:));
    m.minThrottle = min(u(2,:)); m.maxThrottle = max(u(2,:));
    % The team used rounded angle bounds 0.0873 and 0.5236 radians.
    m.constraintViolation = max([0;max(x(2,:)-3.5);max(-0.5-x(2,:)); ...
        max(abs(x(3,:))-0.0873);max(abs(u(1,:))-0.5236);max(abs(u(2,:))-1)]);
    if isfield(r,'otherCar')
        separation = r.otherCar.X(1:2,:)-x(1:2,:);
        m.minEllipseValue = min(separation(1,:).^2/144+separation(2,:).^2/9);
        m.minLongitudinalGap = min(separation(1,:));
    end
    s.metrics = m; all{i} = s;
    writeFile(fullfile(dataDir,[name '.json']),jsonencode(s));
end
template = fileread(fullfile(root,'car_project','visualization','replay.template.html'));
writeFile(fullfile(root,'docs','replay.html'),strrep(template,'__SIMULATION_DATA__',jsonencode([all{:}])));
end

function writeFile(path,content)
fid = fopen(path,'w','n','UTF-8'); assert(fid ~= -1,'Cannot open %s',path);
cleanup = onCleanup(@() fclose(fid)); fprintf(fid,'%s',content);
end
