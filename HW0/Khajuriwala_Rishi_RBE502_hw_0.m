
%% Question 1 Torque equation 

syms m1 m2 q1 q2 l1 l2 dq1 dq2 g Dq1 Dq2  th1(t) th2(t) 
t = sym('t','real');
syms t1 t2 t2 t1(t) t2(t) real;
assume(t1(t),'real');
assume(t2(t),'real');
% here in this question
% q = joint angle
% dq = q dot
% Dq = q double dot
%% End effector position
% here x(1) is actually l*cos(pi/2-q) some i have taken it as l*sinq 
x1 =[l1*sin(q1);
     -l1*cos(q1);];
x2 = [l1*sin(q1) - l2*sin(q1+q2);
    -l1*cos(q1) + l2*cos(q1+q2)]; 

%% Velocity
v1 = jacobian(x1,q1)* (dq1);
v2 = jacobian(x2,[q1 q2])* [dq1; dq2];
%% Kinetic Energy
k1 = 0.5 * m1 * (v1.' * v1);
k2 = 0.5 * m2 * (v2.' * v2);
%% Potential Energy
p1 = -1* m1*g*l1*cos(q1);
p2 = -m2*g*l1*cos(q1) + m2*g*l2*cos(q1+q2)
k = k1 + k2;
p = p1 + p2;
%% lagrangian (L)
L = simplify(k-p)
%% euler- langrange equation
% tau =  d/dt(dL/d(dq)) -dL/dq
%% For the first part of the equation i.e d/dt(dL/d(dq))

d_L_dq_1 = diff(L,dq1);
L_dq_1_t = subs(d_L_dq_1, [q1 q2 dq1 dq2], [t1 t2 diff(t1(t),t) diff(t2(t),t)]);
d_L_dq_1_t = diff(L_dq_1_t, t);
d_L_dq_1 = subs(d_L_dq_1_t, [t1 t2 diff(t1(t),t) diff(t2(t),t) diff(t1(t),t,t) diff(t2(t),t,t)], [q1 q2 dq1 dq2 Dq1 Dq2]);

% for the second part of the equation i.e dL/dq
d_L_q1 = diff(L,q1);
% tau1
Tau_1 = simplify(d_L_dq_1 - d_L_q1)
%% for tau2
% for the first part of the equation i.e d/dt(dL/d(dq))

d_L_dq_2 = diff(L,dq2);
L_dq_2_t = subs(d_L_dq_2, [q1 q2 dq1 dq2], [t1 t2 diff(t1(t),t) diff(t2(t),t)]);
d_L_dq_2_t = diff(L_dq_2_t, t);
d_L_dq_2 = subs(d_L_dq_2_t, [t1 t2 diff(t1(t),t) diff(t2(t),t) diff(t1(t),t,t) diff(t2(t),t,t)],[q1 q2 dq1 dq2 Dq1 Dq2]);

% for the second part of the equation i.e dL/dq
d_L_q2 = diff(L,q2);
% tau2
Tau_2 = simplify(d_L_dq_2 - d_L_q2)
