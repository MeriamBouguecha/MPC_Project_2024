function options = project_solver
% Keep Gurobi support, with MATLAB quadprog as a license-light fallback.
if exist('gurobi','file') ~= 0
    solver = 'gurobi';
elseif exist('quadprog','file') == 2
    solver = 'quadprog';
else
    error('Setup:QPSolver','Install Gurobi or MATLAB Optimization Toolbox.');
end
options = sdpsettings('solver',solver,'verbose',0);
end
