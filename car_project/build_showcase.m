function results = build_showcase(includeLinear)
% Reproduce the portfolio replay from the team's recovered NMPC controllers.
% Run in a fresh MATLAB session after adding CasADi and car_project to path.
if nargin < 1, includeLinear = false; end
names = {'nonlinear-tracking','nonlinear-avoidance'};
if includeLinear, names = [names {'tracking','offset-free'}]; end
results = cell(size(names));
for i = 1:numel(names)
    results{i} = run_project(names{i},false);
end
export_project_replay(results,names);
end
