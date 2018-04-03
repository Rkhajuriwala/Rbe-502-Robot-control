% Notations: For a given variable, x, dx is its time derivative, ddx is
% 2nd-order derivative. 
clc
clear all;
close all;
% the following parameters for the arm
I1=10;  I2 = 10; m1=5; r1=.5; m2=5; r2=.5; l1=1; l2=1;

% we compute the parameters in the dynamic model
a = I1+I2+m1*r1^2+ m2*(l1^2+ r2^2);
b = m2*l1*r2;
d = I2+ m2*r2^2;


%% create symbolic variable for x.
% x1 - theta1
% x2 - theta2

symx= sym('symx',[4,1]); 

M = [a+2*b*cos(symx(2)), d+b*cos(symx(2));
    d+b*cos(symx(2)), d];
C = [-b*sin(symx(2))*symx(4), -b*sin(symx(2))*(symx(3)+symx(4)); b*sin(symx(2))*symx(3),0];
invM = inv(M);
invMC= invM*C;

% the options for ode
% initial condition
x0= [-0.5,0.2,0.1,0.1]; % [q1(0),q2(0),q1_dot(0),q2_dot(0) ]
w=0.2;
tf = 5;
xf = [0, 0, 0, 0]';



%% Implement the PD control for set point tracking.

options = odeset('RelTol',1e-4,'AbsTol',[1e-4, 1e-4, 1e-4, 1e-4]);
[T,X] = ode45(@(t,x) PDControl(t,x,xf,symx,invM,invMC),[0 tf],x0, options);


figure('Name','Theta_1 under PD SetPoint Control');
plot(T, X(:,1),'r-');
hold on

figure('Name','Theta_2 under PD SetPoint Control');
plot(T, X(:,2),'r--');
% hold on

%% Implement the inverse dynamic control.
options = odeset('RelTol',1e-4,'AbsTol',[1e-4, 1e-4, 1e-4, 1e-4]);
[T,X] = ode45(@(t,x)inverseDynamicControl(t,x),[0 tf],x0, options);

figure('Name','Theta_1 under Computed Torque Control');
plot(T, X(:,1),'r-');
hold on
plot(T, w*ones(size(T,1),1),'b-');
figure('Name','Theta_2 under Computed Torque Control');
plot(T, X(:,2),'r--');
hold on
plot(T, +sin(2*T),'b-');
%%function for PD
function dx = PDControl(t,x,xf,symx,invM,invMC)

    Kp = [1000 0;0 1000];
    Kv = [100 0;0 100];
    i_m = subs(invM,[symx(1) symx(2) symx(3) symx(4)],[x(1) x(2) x(3) x(4)]);
    i_m_c = subs(invMC,[symx(1) symx(2) symx(3) symx(4)],[x(1) x(2) x(3) x(4)]);
    u = -Kp*((x(1:2)-xf(1:2))) - Kv*((x(3:4)-xf(3:4)));
    dx(1) = x(3)
    dx(2) = x(4)
    dx(3:4) = i_m*u - i_m_c*x(3:4);
    dx = dx';
end


%%function for InverseDynamicControls
function dx = inverseDynamicControl(t,x)    
   xf = [ 0.2, sin(2*t), 0, 2*cos(2*t) ]';
   Kp = [1000 0;0 1000];
   Kv = [100 0;0 100];
   a_d = [ 0, -4*sin(2*t) ]';
   a = -Kp*((x(1:2)-xf(1:2))) - Kv*((x(3:4)-xf(3:4))) + a_d;
   dx(1) = x(3);
   dx(2) = x(4);
   dx(3:4) = a;
   dx = dx';
end
