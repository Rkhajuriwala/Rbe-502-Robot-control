clc;
close all;
clear all;
% Initial and final condition (x, y, theta)
% q0 = [0, 4, 0];
% q0 = [0, 5, 0];
q0 = [-1, 4.5, 0];
qi = [6, 2, 0];
qf = [0, 0, 0];

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
l=1;
syms v w theta x y phi l
f= [v*cos(theta); v*sin(theta); (v/l)*tan(phi)];
dfdx = jacobian(f,  [x;y;theta]);
dfdu = jacobian(f,  [v;phi]);

% solve linear equations for finding the coefficients in the feasible
% trajectories.

% % initial and terminal condition: with velocity equals zero.
% [matA,matb] = equationsToMatrix([x0==q0(1), y0==q0(2), dx0*sin(q0(3))+  dy0*cos(q0(3))==1, dx0*cos(q0(3))+ dy0*sin(q0(3))==1, ...
% xf==qf(1), yf==qf(2), dxf*sin(qf(3))+ dyf*cos(qf(3))==1, dxf*cos(qf(3))+ dyf*sin(qf(3))==1],[a(1),a(2),a(3),a(4),b(1),b(2),b(3),b(4)]);
% initial and terminal condition: with velocity equals zero.
[matA1,matb1] = equationsToMatrix([x0==q0(1), y0==q0(2), dx0*sin(q0(3))-  dy0*cos(q0(3))==0, dx0*sin(q0(3))+ dy0*cos(q0(3))==0, ...
xf==qi(1), yf==qi(2), dxf*sin(qi(3))- dyf*cos(qi(3))==0, dxf*sin(qi(3))+ dyf*cos(qi(3))==0],[a(1),a(2),a(3),a(4),b(1),b(2),b(3),b(4)]);
param1 = matA1\matb1;
avec1= double(param1(1:4)');
bvec1 = double(param1(5:end)');

[matA2,matb2] = equationsToMatrix([x0==qi(1), y0==qi(2), dx0*sin(qi(3))-  dy0*cos(qi(3))==0, dx0*sin(qi(3))+ dy0*cos(qi(3))==0, ...
xf==qf(1), yf==qf(2), dxf*sin(qf(3))- dyf*cos(qf(3))==0, dxf*sin(qf(3))+ dyf*cos(qf(3))==0],[a(1),a(2),a(3),a(4),b(1),b(2),b(3),b(4)]);
param2 = matA2\matb2;
avec2= double(param2(1:4)');
bvec2 = double(param2(5:end)');


% now apply the feedback controller
% ode_tracking;
figure
Xdes1 = ode_tracking(Tf,avec1,bvec1);
plot(Xdes1(1,:), Xdes1(2,:), 'LineWidth', 4);
hold on 
Xdes2 = ode_tracking(Tf,avec2,bvec2);
plot(Xdes2(1,:), Xdes2(2,:), 'LineWidth', 4);
function Xdes = ode_tracking(Tf,avec, bvec)
% evaluate the desired state.
dt=0.01;
tsteps=[0:dt:Tf];
N=size(tsteps,2);
X = zeros(3,N);

% with some initial error
% with no-initial error 
X(:,1)=[0, 2, pi/6];

Xdes = zeros(3,N);

for i=1:N-1
    xvec = X(:,i);
x = xvec(1);
y = xvec(2);
theta = xvec(3);
theta= wrapTo2Pi(theta);
%theta = theta - 2*pi*floor(theta/(2*pi));


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
% A =  [ 0, 0,  vf*cos(thetades);
%  0, 0, -vf*sin(thetades);
%  0, 0,             0];
% B = [ sin(thetades),                    0;
%  cos(thetades),                    0;
%  tan(deltaf), vf*(tan(deltaf)^2 + 1)]; 


A = [ 0, 0, -vf*sin(thetades);
    0, 0,  vf*cos(thetades);
    0, 0,             0];
 
B = [ cos(thetades), 0;
    sin(thetades), 0;
    0, 1];

Q= eye(3);
R = eye(2);
%if any(eig(A-B*K))>=0;
K= lqr(A,B,Q,R);
%end

u = -K*(xvec - xdes_vec) + [vf; wf];

dxvec = [u(1)*cos(theta);u(1)*sin(theta);u(2)];
% 
% % without noise
 X(:,i+1)= dxvec*dt+ X(:,i);

% with noise
%X(:,i+1)= dxvec*dt+ X(:,i) +0.05*randn(1);


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
