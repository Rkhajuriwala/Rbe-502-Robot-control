% Notations: For a given variable, x, dx is its time derivative, ddx is
% 2nd-order derivative. 
clc
clear all;
close all;
% the following parameters for the arm
I1=10;  I2 = 10; m1=5; r1=.5; m2=5; r2=.5; l1=1; l2=1; g=9.8;
 
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
invMC= inv(M)*C;

%% Gravity Matrix 
g1=-(m1+m2)*g*l1*sin(symx(2))-m2*g*l2*sin(symx(1)+symx(2)); 
g2=-m2*g*l2*sin(symx(1)+symx(2)); 
Gq=[g1;g2]; 


%% specify your initial and final condition.
x0= [-0.5,0.2,0.1,0.1]; % same as the intial condition provided in the Midterm(too lazy to choose other)
w=0.2;
tf = 10;
xf = [-1.5,1.5, 0, 0];
% xf = [0, 0, 0, 0]';
%% for values of alpha for determing Kp
temp = jacobian(Gq,[symx(1) symx(2)]);
alpha = vpa(norm(subs(temp,[symx(1) symx(2)],[x0(1) x0(2)])));



%% Plot for Augmented PD control
options = odeset('RelTol',1e-4,'AbsTol',[1e-4, 1e-4, 1e-4, 1e-4]);
[T,X] = ode45(@(t,x)augmentedPD(t,x,symx,invM,invMC),[0 tf],x0, options);

figure('Name','Theta_1 under Augmented PD control');
plot(T, X(:,1),'r-');
hold on
plot(T, w*ones(size(T,1),1),'b-');
figure('Name','Theta_2 under Augmented PD control');
plot(T, X(:,2),'r--');
hold on
plot(T, +sin(2*T),'b-');

%% Plot for iterative learning
options = odeset('RelTol',1e-4,'AbsTol',[1e-4, 1e-4, 1e-4, 1e-4]);
[T1,X1] = ode45(@(t,x)iterative_learning(t,x,symx,invM,invMC,Gq,xf),[0 tf],x0, options);

figure('Name','Theta_1 under Computed Torque Control');
plot(T1, X1(:,1),'r-');
hold on
plot(T1, -1.5*ones(size(T1,1),1),'b-');
figure('Name','Theta_2 under Computed Torque Control');
plot(T1, X1(:,2),'r--');
hold on
plot(T1, 1.5*ones(size(T1,1),1),'b-');
%% function for Augmented PD
function dx = augmentedPD(t,x,symx,invM,invMC)
%     x = [-0.5,0.2,0.1,0.1]
    dx = zeros(4,1);
  
    q1 = x(1);
    q2 = x(2);
    q1_dot = x(3);
    q2_dot =x(4);
    Kp = [250 0;0 250];
    Kv = [50 0; 0 50];
    i_m = subs(invM,[symx(1) symx(2) symx(3) symx(4)],[q1 q2 q1_dot q2_dot]);
    i_m_c = subs(invMC,[symx(1) symx(2) symx(3) symx(4)],[q1 q2 q1_dot q2_dot]);
    x = [q1;q2];
    x_dot = [q1_dot;q2_dot];
    xd = [0.2;sin(2*t)];
    xd_dot = [0;2*cos(2*t)];
    xD_dot = [0;-4*sin(2*t)];
    dx(1) = x_dot(1);
    dx(2) = x_dot(2);
    dx(3:4) = -i_m*Kp*(x-xd) - i_m*Kv*(x_dot-xd_dot) - i_m_c*(x_dot-xd_dot)+ xD_dot;
end
%% Function for Iterative learning
function dx = iterative_learning(t,x,symx,invM,invMC,Gq,xf)

    dx = zeros(4,1);
  
    q1 = x(1);
    q2 = x(2);
    q1_dot = x(3);
    q2_dot =x(4);
    q1_d = xf(1);
    q2_d = xf(2);
    q1_d_dot = xf(3);
    q2_d_dot = xf(4);
    x1 = [q1;q2];
    x1_dot =[q1_dot;q2_dot];
    xd = [q1_d;q2_d];
    xd_dot =[q1_d_dot;q2_d_dot];

    Kp = [200 0;0 200];
    Kv = [50 0; 0 50];
    i_m = subs(invM,[symx(1) symx(2) symx(3) symx(4)],[q1 q2 q1_dot q2_dot]);
    i_m_c = subs(invMC,[symx(1) symx(2) symx(3) symx(4)],[q1 q2 q1_dot q2_dot]);
    i_n = subs(Gq,[symx(1) symx(2) symx(3) symx(4)],[q1 q2 q1_dot q2_dot]);
    beta = 0.1;
    global u_p
    if t == 0
        u_p = [0;0];
    end  
    u = (1/beta)*(-Kp*(x1-xd)) - Kv*((x1_dot-xd_dot))+u_p;
%     Update Law
    if norm(x1_dot) < 0.0001
%         u_p = (1/beta)*(-Kp*(x-xd));
        u_p = u;
    end
       
    
    dx(1:2) = x1_dot;
    dx(3:4) = i_m*u - i_m_c*(x1_dot) - i_m*i_n;

end
