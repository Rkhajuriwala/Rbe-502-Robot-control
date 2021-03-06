function []= robustControl(theta10,theta20,dtheta10, dtheta20,theta1f, theta2f,dtheta1f,dtheta2f,tf)

% Robust control design for 2-D planar arm.
% input: initial and final state.
% output: Demostrate the performance of robust controller with parameter
% uncertainty.
% the nominal model parameter:
m1 =10; m2=5; l1=1; l2=1; r1=0.5; r2 =.5; I1=10/12; I2=5/12; % parameters in the paper.
% the nominal parameter vector b0 is
b0 = [ m1* r1^2 + m2*l1^2 + I1; m2*r2^2 + I2; m2*l1*r2];


%% Trajectory planning block
% Initial condition (TODO: CHANGE DIFFERENT INITIAL AND FINAL STATES)
x0=[-0.5,0,-1,0.1];
x0e = [-0.7,0.5,-0.2,0]; % an error in the initial state.
xf=[0.8,0, 0.5, 0];
% The parameter for planned joint trajectory 1 and 2.
global a1 a2 % two polynomial trajectory for the robot joint
nofigure=1;
% Traj generation.
a1 = planarArmTraj(theta10,dtheta10, theta1f, dtheta1f,tf, nofigure);
a2 = planarArmTraj(theta20,dtheta20, theta2f, dtheta2f,tf, nofigure);


torque=[];
options = odeset('RelTol',1e-4,'AbsTol',[1e-4, 1e-4, 1e-4, 1e-4]);
%% TODO: IMPLEMENT THE CONTROLLER
 [T,X] = ode45(@(t,x)planarArmODERobust(t,x),[0 tf],x0e,options);
figure('Name','theta1');
plot(T, X(:,1),'r-');
hold on
plot(T, a1(1)+a1(2)*T+ a1(3)*T.^2+a1(4)*T.^3,'b-')
title('Theta_1 under Robust Control');
figure('Name','theta2');
plot(T, X(:,2),'r-');
hold on
plot(T, a2(1)+a2(2)*T+ a2(3)*T.^2+a2(4)*T.^3, 'b-');
title('Theta_2 under Robust Control');
%% TODO: IMPLEMENT THE CONTROLLER TO AVOID CHATTERING.
[T1,X1] = ode45(@(t,x)planarArmODERobustApprx(t,x),[0 tf],x0e,options);

figure('Name','theta1');
plot(T1, X1(:,1),'r-');
hold on
plot(T1, a1(1)+a1(2)*T1+ a1(3)*T1.^2+a1(4)*T1.^3,'b-')
title('Theta_1 under Robust Control (Avoiding chattering)');
figure('Name','theta2');
plot(T1, X1(:,2),'r-');
hold on
plot(T1, a2(1)+a2(2)*T1+ a2(3)*T1.^2+a2(4)*T1.^3, 'b-');
title('Theta_2 under Robust Control(Avoiding chattering)');

%TODO: PLOT THE INPUT TRAJECTORY




    function [dx ] = planarArmODERobust(t,x)
        %Todo: Select your feedback gain matrix Kp and Kd.
        Kp = eye(2)*1000;
        Kd = eye(2)*500;
        % Compute the desired state and their time derivatives from planned
        % trajectory.
        vec_t = [1; t; t^2; t^3]; % cubic polynomials
        theta_d= [a1'*vec_t; a2'*vec_t];
        %ref = [ref,theta_d];
        % compute the velocity and acceleration in both theta 1 and theta2.
        a1_vel = [a1(2), 2*a1(3), 3*a1(4), 0];
        a1_acc = [2*a1(3), 6*a1(4),0,0 ];
        a2_vel = [a2(2), 2*a2(3), 3*a2(4), 0];
        a2_acc = [2*a2(3), 6*a2(4),0,0 ];
        dtheta_d =[a1_vel*vec_t; a2_vel* vec_t];
        ddtheta_d =[a2_acc*vec_t; a2_acc* vec_t];
        theta= x(1:2,1);
        dtheta= x(3:4,1);

        %% Calculating the Mbar
        % M upper bound
        a_u = I1+I2+(15/12)+m1*r1^2+ (m2+10)*(l1^2+ (r2+0.5)^2);
        b_u = (m2+10)*l1*(r2+0.5);
        d_u = I2+(15/12)+ (m2+10)*(r2+0.5)^2;
        M_u = [a_u+2*b_u*cos(x(2)), d_u+b_u*cos(x(2));  d_u+b_u*cos(x(2)), d_u];
        C_u = [-b_u*sin(x(2))*x(4), -b_u*sin(x(2))*(x(3)+x(4)); b_u*sin(x(2))*x(3),0];
        % M lower Bound
        a_l = I1+I2+m1*r1^2+ m2*(l1^2+ r2^2);
        b_l = m2*l1*r2;
        d_l = I2+m2*r2^2;
        M_l = [a_l+2*b_l*cos(x(2)), d_l+b_l*cos(x(2));  d_l+b_l*cos(x(2)), d_l];
        C_l = [-b_l*sin(x(2))*x(4), -b_l*sin(x(2))*(x(3)+x(4)); b_l*sin(x(2))*x(3),0];
        % M_bar = 2/(Mu+Ml)
        M_bar = 2* inv(M_u + M_l);
        b_bar = (M_bar(2,1)-M_bar(2,2))/cos(x(2));
        C_bar = [-b_bar*sin(x(2))*x(4), -b_bar*sin(x(2))*(x(3)+x(4)); b_bar*sin(x(2))*x(3),0];
        %the true model
        m2t = m2+ 10*rand(1);% m1 true value is in [m1, m1+epsilon_m1] and epsilon_m1 a random number in [0,10];
        r2t = r2 + 0.5*rand(1);
        I2t = I2 + (15/12)*rand(1);

        a = I1+I2+m1*r1^2+ m2t*(l1^2+ r2t^2);
        b = m2t*l1*r2t;
        d = I2t+ m2t*r2t^2;


        % the actual dynamic model of the system is characterized by M and
        % C
        Mmat = [a+2*b*cos(x(2)), d+b*cos(x(2));  d+b*cos(x(2)), d];
        Cmat = [-b*sin(x(2))*x(4), -b*sin(x(2))*(x(3)+x(4)); b*sin(x(2))*x(3),0];
        invM = inv(Mmat);
        invMC = invM*Cmat;

        %% Calculating Value of V for the controller
        e = theta - theta_d;
        e_dot = dtheta - dtheta_d;
        xe1 = [e(1),e_dot(1)];
        xe2 = [e(2),e_dot(2)];
        P = [1 0; 0 1];
        B = [0; 1];
        gamma_1 = 0.5;
        gamma_2 = 0.5;
        gamma_3 = 0.5;
        alpha = 0;
        rho1 = (1/(1-alpha))*(gamma_1*norm(xe1)+gamma_2*(norm(xe1).^2)+gamma_3);
        rho2 = (1/(1-alpha))*(gamma_1*norm(xe2)+gamma_2*(norm(xe2).^2)+gamma_3);
        w = [xe1*P*B;xe2*P*B];
        if norm(w) ~=0
          v = [-xe1*P*B*rho1/norm(xe1*P*B);-xe2*P*B*rho2/norm(xe2*P*B)];
        else
          v = [0;0];
        end
        %% TODO: compute the robust controller
        aq = ddtheta_d - Kp*e - Kd*e_dot + v;
        q_d_dot=(invM*(M_bar*aq + C_bar*dtheta));
        %TODO: update the system state, compute dx
        dx=zeros(4,1);
        dx(1) = x(3);
        dx(2) = x(4);
        dx(3) = q_d_dot(1);
        dx(4) = q_d_dot(2);
%         dx(3:4) = -invMC* x(3:4) +invM*tau; % because ddot theta = -M^{-1}(C \dot Theta) + M^{-1} tau
    end



        function [dx ] = planarArmODERobustApprx(t,x)
        %Todo: Select your feedback gain matrix Kp and Kd.
        Kp = eye(2)*1000;
        Kd = eye(2)*500;
        % Compute the desired state and their time derivatives from planned
        % trajectory.
        vec_t = [1; t; t^2; t^3]; % cubic polynomials
        theta_d= [a1'*vec_t; a2'*vec_t];
        %ref = [ref,theta_d];
        % compute the velocity and acceleration in both theta 1 and theta2.
        a1_vel = [a1(2), 2*a1(3), 3*a1(4), 0];
        a1_acc = [2*a1(3), 6*a1(4),0,0 ];
        a2_vel = [a2(2), 2*a2(3), 3*a2(4), 0];
        a2_acc = [2*a2(3), 6*a2(4),0,0 ];
        dtheta_d =[a1_vel*vec_t; a2_vel* vec_t];
        ddtheta_d =[a2_acc*vec_t; a2_acc* vec_t];
        theta= x(1:2,1);
        dtheta= x(3:4,1);

        %the true model
        m2t = m2+ 10*rand(1);% m1 true value is in [m1, m1+epsilon_m1] and epsilon_m1 a random number in [0,10];
        r2t = r2 + 0.5*rand(1);
        I2t = I2 + (15/12)*rand(1);

        a = I1+I2+m1*r1^2+ m2t*(l1^2+ r2t^2);
        b = m2t*l1*r2t;
        d = I2t+ m2t*r2t^2;
        % the actual dynamic model of the system is characterized by M and
        % C
        Mmat = [a+2*b*cos(x(2)), d+b*cos(x(2));  d+b*cos(x(2)), d];
        Cmat = [-b*sin(x(2))*x(4), -b*sin(x(2))*(x(3)+x(4)); b*sin(x(2))*x(3),0];
        invM = inv(Mmat);
        invMC = invM*Cmat;
        % M upper bound
        a_u = I1+I2+(15/12)+m1*r1^2+ (m2+10)*(l1^2+ (r2+0.5)^2);
        b_u = (m2+10)*l1*(r2+0.5);
        d_u = I2+(15/12)+ (m2+10)*(r2+0.5)^2;
        M_u = [a_u+2*b_u*cos(x(2)), d_u+b_u*cos(x(2));  d_u+b_u*cos(x(2)), d_u];
        C_u = [-b_u*sin(x(2))*x(4), -b_u*sin(x(2))*(x(3)+x(4)); b_u*sin(x(2))*x(3),0];
        % M lower Bound
        a_l = I1+I2+m1*r1^2+ m2*(l1^2+ r2^2);
        b_l = m2*l1*r2;
        d_l = I2+m2*r2^2;
        M_l = [a_l+2*b_l*cos(x(2)), d_l+b_l*cos(x(2));  d_l+b_l*cos(x(2)), d_l];
        C_l = [-b_l*sin(x(2))*x(4), -b_l*sin(x(2))*(x(3)+x(4)); b_l*sin(x(2))*x(3),0];
        % M_bar = 2/(Mu+Ml)
        M_bar = 2* inv(M_u + M_l);
        b_bar = (M_bar(2,1)-M_bar(2,2))/cos(x(2));
        C_bar = [-b_bar*sin(x(2))*x(4), -b_bar*sin(x(2))*(x(3)+x(4)); b_bar*sin(x(2))*x(3),0];
        %% Calculating value of V
        e = theta - theta_d;
        e_dot = dtheta - dtheta_d;
        xe1 = [e(1);e_dot(1)];
        xe2 = [e(2);e_dot(2)];
        P = [1 0;0 1];
        B = [0;1];
        gamma_1 = 0.5;
        gamma_2 = 0.5;
        gamma_3 = 0.5;
        alpha = 0;
        rho1 = (1/(1-alpha))*(gamma_1*norm(xe1') + (gamma_2)*(norm(xe2').^2)+gamma_3);
        rho2 = (1/(1-alpha))*(gamma_1*norm(xe2') + (gamma_2)*(norm(xe2').^2)+gamma_3);
        rho = [rho1,0;
              0,rho2];
        w = [B'*P*xe1;B'*P*xe2];% according to the slides
        epsilon = 0.2;
        if norm(w)> epsilon
            v = - rho*(w/norm(w));
        else
            v = -rho*(w/epsilon);
        end
        %TODO: compute the robust controller to avoid chattering.
        aq = ddtheta_d - Kp*e - Kd*e_dot + v;
        q_d_dot=(invM*(M_bar*aq + C_bar*dtheta));
        %TODO: update the system state, compute dx
        dx=zeros(4,1);
        dx(1) = x(3);
        dx(2) = x(4);
        dx(3) = q_d_dot(1);
        dx(4) = q_d_dot(2);
%         dx(3:4) = -invMC* x(3:4) +invM*tau; % because ddot theta = -M^{-1}(C \dot Theta) + M^{-1} tau
        end
end
