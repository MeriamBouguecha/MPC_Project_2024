function [K,P,e] = project_dlqr(A,B,Q,R)
% Use MATLAB dlqr when available; otherwise solve the same DARE by iteration.
% This is a portability helper, not a different controller design.
if exist('dlqr','file') ~= 0
    [K,P,e] = dlqr(A,B,Q,R); return;
end
P = Q;
for i = 1:10000
    K = (R+B'*P*B)\(B'*P*A);
    next = Q+A'*P*A-A'*P*B*K;
    next = (next+next')/2;
    if norm(next-P,'fro') <= 1e-11*max(1,norm(P,'fro'))
        P = next;
        K = (R+B'*P*B)\(B'*P*A);
        e = eig(A-B*K); return;
    end
    P = next;
end
error('Project:Riccati','DARE iteration did not converge.');
end
