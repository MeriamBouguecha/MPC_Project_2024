function R = mpt_call_linprog(S)
% Modern MATLAB adapter for MPT3's solver interface.
% Older MPT3 releases pass a removed x0 argument to linprog. This adapter
% keeps MPT's result contract without modifying the installed dependency.
global MPTOPTIONS
if isempty(MPTOPTIONS) && ~S.test, MPTOPTIONS = mptopt; end
assert(strcmpi(S.problem_type,'LP'),'MPT adapter expects an LP.');
options = optimoptions('linprog','Display','none');
[R.xopt,R.obj,flag,~,R.lambda] = linprog(S.f,S.A,S.b,S.Ae,S.be,S.lb,S.ub,options);
if flag > 0
    R.how = 'ok';
    if S.test, R.exitflag = 1; else, R.exitflag = MPTOPTIONS.OK; end
elseif flag == -3
    R.how = 'unbounded';
    if S.test, R.exitflag = 3; else, R.exitflag = MPTOPTIONS.UNBOUNDED; end
else
    R.how = 'infeasible';
    if S.test, R.exitflag = 2; else, R.exitflag = MPTOPTIONS.INFEASIBLE; end
end
end
