function tests = test_team_project
tests = functiontests(localfunctions);
end

function setupOnce(t)
here = fileparts(mfilename('fullpath'));
addpath(fileparts(here));
setup_project('nonlinear-tracking');
t.TestData.car = Car(0.1);
end

function testModelAndLinearization(t)
c = t.TestData.car; [x,u] = c.steady_state(120/3.6);
f = c.f(x,u); verifyEqual(t,f(2:4),zeros(3,1),'AbsTol',1e-8);
sys = c.linearize(x,u); eps = 1e-5;
A = zeros(4); B = zeros(4,2);
for i = 1:4
    e = zeros(4,1); e(i) = eps;
    A(:,i) = (c.f(x+e,u)-c.f(x-e,u))/(2*eps);
end
for i = 1:2
    e = zeros(2,1); e(i) = eps;
    B(:,i) = (c.f(x,u+e)-c.f(x,u-e))/(2*eps);
end
verifyEqual(t,sys.A,A,'AbsTol',1e-7);
verifyEqual(t,sys.B,B,'AbsTol',1e-7);
end

function testRiccatiHelper(t)
A = [1 .1;0 1]; B = [.005;.1]; Q = diag([1,10]); R = 2;
[K,P,e] = project_dlqr(A,B,Q,R);
residual = Q+A'*P*A-A'*P*B*((R+B'*P*B)\(B'*P*A))-P;
verifyLessThan(t,norm(residual,'fro'),1e-7);
verifyLessThan(t,max(abs(e)),1);
verifyEqual(t,K,(R+B'*P*B)\(B'*P*A),'AbsTol',1e-9);
end

function testNonlinearTracking(t)
r = run_project('nonlinear-tracking',false);
verifyRun(t,r);
verifyLessThan(t,abs(r.myCar.X(2,end)-3),.01);
verifyLessThan(t,abs(r.myCar.X(4,end)*3.6-100),.01);
end

function testNonlinearAvoidance(t)
r = run_project('nonlinear-avoidance',false);
verifyRun(t,r);
d = r.myCar.X(1:2,:)-r.otherCar.X(1:2,:);
verifyGreaterThanOrEqual(t,min(d(1,:).^2/144+d(2,:).^2/9),1-1e-6);
verifyGreaterThan(t,r.myCar.X(1,end),r.otherCar.X(1,end));
verifyLessThan(t,abs(r.myCar.X(2,end)),.01);
% Every constrained prediction state must be connected to a control interval.
ctrl = NmpcControl(t.TestData.car,3);
verifyEqual(t,size(ctrl.X,2),size(ctrl.U,2)+1);
end

function testLinearAndOffsetFree(t)
for name = {'tracking','offset-free'}
    r = run_project(name{1},false);
    verifyRun(t,r);
    verifyLessThan(t,abs(r.myCar.X(2,end)-3),.1);
    verifyLessThan(t,abs(r.myCar.X(4,end)-r.myCar.Ref(2,end))*3.6,.5);
end
end

function testDynamicsAndDomain(t)
r = run_project('dynamics',false);
verifyEqual(t,r.T(end),2);
verifyGreaterThan(t,r.myCar.X(4,end),r.myCar.X(4,1));
verifyTrue(t,all(r.myCar.U(2,:)==.7));
end

function verifyRun(t,r)
x = r.myCar.X; u = r.myCar.U;
verifyEqual(t,r.T(end),15);
verifyTrue(t,all(isfinite(x(:))));
verifyLessThanOrEqual(t,max(abs(u(2,:))),1+1e-7);
verifyLessThanOrEqual(t,max(abs(u(1,:))),pi/6+1e-7);
verifyGreaterThanOrEqual(t,min(x(2,:)),-.5-1e-5);
verifyLessThanOrEqual(t,max(x(2,:)),3.5+1e-5);
verifyLessThanOrEqual(t,max(abs(x(3,:))),.0873+1e-5);
end
