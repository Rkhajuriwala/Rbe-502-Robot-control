%% Question 1

clc;
close all;
clear all;
% Initial and final condition (x, y, theta)
q0 = [0, 4, 0,0];
% q0 = [0, 5, 0,0];
% q0 = [-1, 4.5, 0,0];
qi = [6, 2, 0,0];
qf = [0, 0, 0,0];

Tf = 10;				% time to reach destination

syms t
a = sym('a', [1,4]); % the parameters of trajectory for x
b = sym('b', [1,4]); % the parameters of trajectory for y
basis = [1; t; t^2; t^3];
dbasis = [0; 1; 2*t; 3*t^2];
xsym = a*basis;
ysym = b*basis;
dx = a*dbasis;
dy = b*dbasis;

x0 = subs(xsym,t,0);
xf = subs(xsym,t,Tf);
dx0 = subs(dx,t,0);
dxf = subs(dx,t,Tf);

y0 = subs(ysym,t,0);
yf = subs(ysym,t,Tf);
dy0 = subs(dy,t,0);
dyf = subs(dy,t,Tf);

% compute the jacobian linearization of the vector field.
% l=1;
syms v w theta x y phi l

f= [v*cos(theta); v*sin(theta); (v/l)*tan(phi)];
dfdx = jacobian(f,[x;y;theta]);
dfdu = jacobian(f,[v;phi]);

% solve linear equations for finding the coefficients in the feasible
% trajectories.

% initial and terminal condition: with velocity equals zero.
[matA1,matb1] = equationsToMatrix([x0==q0(1), y0==q0(2), dx0*sin(q0(3))-  dy0*cos(q0(3))==0, dx0*cos(q0(4))+ dy0*sin(q0(4))==0, ...
xf==qi(1), yf==qi(2), dxf*sin(qi(3))- dyf*cos(qi(3))==0, dxf*cos(qi(4))+ dyf*sin(qi(4))==0],[a(1),a(2),a(3),a(4),b(1),b(2),b(3),b(4)]);
param1 = matA1\matb1;
avec1= double(param1(1:4)');
bvec1 = double(param1(5:end)');

[matA2,matb2] = equationsToMatrix([x0==qi(1), y0==qi(2), dx0*sin(qi(3))-  dy0*cos(qi(3))==0, dx0*cos(qi(4))+ dy0*sin(qi(4))==0, ...
xf==qf(1), yf==qf(2), dxf*sin(qf(3))- dyf*cos(qf(3))==0, dxf*cos(qf(4))+ dyf*sin(qf(4))==0],[a(1),a(2),a(3),a(4),b(1),b(2),b(3),b(4)]);
param2 = matA2\matb2;
avec2= double(param2(1:4)');
bvec2 = double(param2(5:end)');


% now apply the feedback controller
% ode_tracking;

% hold on 

% plot(X2(1,:), X2(2,:),'LineWidth', 4);
% figure;
% plot(X1(1,:), X1(2,:),'LineWidth', 4);
% hold on
% plot(X2(1,:), X2(2,:),'LineWidth', 4);
% [qo1,qo2] = [0,4];
% [qi1,qi2] = [6,2];
% [qf1,qf2] = [0,0];
vo1 = 0;
v02 = 0;
vi1 = 0;
vi2 = 0;
vf1 = 0;
vf2 = 0;
d0 = [0,6,vo1,vi1,0,5];
[qd0,vd0,ad0] = cubic(d0(1),d0(2),d0(3),d0(4),d0(5),d0(6));
d1 = [4,2,v02,vi2,0,5];
[qd1,vd1,ad1] = cubic(d1(1),d1(2),d1(3),d1(4),d1(5),d1(6));
d2 = [6,0,vi1,vf1,0,5];
[qd2,vd2,ad2] = cubic(d2(1),d2(2),d2(3),d2(4),d2(5),d2(6));
d3 = [2,0,vi2,vf2,0,5];
[qd3,vd3,ad3] = cubic(d3(1),d3(2),d3(3),d3(4),d3(5),d3(6));
 t = linspace(0,5,100*5);
%% plot-1 Time-position
figure;
% q(1,:) = qd0;
% q(1,251:500) = qd1;
% q(1,501:750) = qd2;
% q(1,751:1000) = qd3;
plot(qd0,qd1,'LineWidth', 4);
title('Trajectory before controller')
hold on
plot(qd2,qd3,'LineWidth', 4);
% hold on
% plot(t,qd2,'LineWidth', 4);
% hold on
% plot(t,qd3,'LineWidth', 4);
figure
[Xdes1,X1] = ode_tracking(Tf,avec1,bvec1);
plot(Xdes1(1,:), Xdes1(2,:), 'LineWidth', 4);
% % hold on 
title('Trajectory after controller')
% title('Trajectory for points (-1,4.5)-->(6,2)-->(0,0)');
hold on 
[Xdes2,X2] = ode_tracking(Tf,avec2,bvec2);
plot(Xdes2(1,:), Xdes2(2,:), 'LineWidth', 4);
function [Xdes,X] = ode_tracking(Tf,avec, bvec)
% evaluate the desired state.
dt=0.01;
tsteps=[0:dt:Tf];
N=size(tsteps,2);
X = zeros(3,N);

% with some initial error
% with no-initial error 
X(:,1)=[0, 4, pi/6];

Xdes = zeros(3,N);

for i=1:N-1
    xvec = X(:,i);
x = xvec(1);
y = xvec(2);
theta = xvec(3);
theta= wrapTo2Pi(theta);
l=1;
t=tsteps(i);
basis = [1; t; t^2; t^3];
dbasis = [0; 1; 2*t; 3*t^2];
ddbasis = [0; 0;2; 6*t];
xdes = avec*basis;
dxdes = avec*dbasis;
ddxdes = avec*ddbasis;

ydes = bvec*basis;
dydes = bvec*dbasis;
ddydes = bvec*ddbasis;

% compute sin(theta_d)

thetades = atan2(dydes, dxdes);
Xdes(:,i)= [xdes;ydes;thetades];

% The desired state.
xdes_vec = [xdes; ydes; thetades];

% compute the feedforward in the input.
vf = dxdes*cos(thetades) + dydes*sin(thetades);
dthetades = 1/vf*(ddydes*cos(thetades) - ddxdes*sin(thetades));
wf = dthetades;


A = [ 0, 0, -vf*sin(thetades);
    0, 0,  vf*cos(thetades);
    0, 0,             0];
 
B = [ cos(thetades), 0;
    sin(thetades), 0;
    0, 1];

Q= eye(3);
R = eye(2);
% for all these values of gain the trajectory is not changing
K = [0 0 1;0 0 1];
% K = [1 0 0;1 0 0];
% K = [1 0 1;1 0 1];
% K = [0.1 0.1 0.1;0.1 0.1 0.1];
% K = [1 1 1;1 1 1];
% K = [10 10 10;10 10 10];
% K = [0 0 0;0 0 0];
% K= lqr(A,B,Q,R);
u = -K*(xvec - xdes_vec) + [vf; wf];
dxvec = [u(1)*cos(theta);u(1)*sin(theta);u(2)];
X(:,i+1)= dxvec*dt+ X(:,i);
end
for i=1:N;
    t=tsteps(i);
    basis = [1; t; t^2; t^3];
dbasis = [0; 1; 2*t; 3*t^2];
ddbasis = [0; 0;2; 6*t];
Xdes(1,i) = avec*basis;
    Xdes(2,i)= bvec*basis;
end
end
 function [qd,vd,ad] =cubic(qi, qf, vi,vf,ti,tf)

t = linspace(ti,tf,100*(tf-ti));
c = ones(size(t)); 
A=[1,ti, ti^2,ti^3;
    0,1,2*ti,3*ti^2;
    1,tf, tf^2,tf^3;
    0,1,2*tf,3*tf^2];

B=[qi;vi;qf;vf];
a=A\B;

qd = a(1).*c + a(2).*t +a(3).*t.^2 + a(4).*t.^3 ;
vd = a(2).*c +2*a(3).*t +3*a(4).*t.^2 ; 
ad = 2*a(3).*c + 6*a(4).*t;
end
