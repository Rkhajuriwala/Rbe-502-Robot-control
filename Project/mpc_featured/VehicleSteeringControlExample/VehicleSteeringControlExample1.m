clc; clear all; close all;
%% Autonomous Vehicle Steering Using Model Predictive Control
% This example shows how to implement an autonomous vehicle steering system
% using model predictive control (MPC).

% Copyright 2017 The MathWorks, Inc.

%%
% Add example file folder to MATLAB(R) path.
addpath(fullfile(matlabroot,'examples','mpc_featured','main'));

%% Lateral Vehicle Dynamics
% To describe the lateral vehicle dynamics, this example uses a _bicycle
% model_ with two degrees of freedom, lateral position and yaw angle. The
% vehicle model is depicted in the following figure.
% 
% <<../VehicleSteeringModel.png>>
%
% Here,
%
% * $X$ and $Y$ denote the position of the vehicle in global coordinates.
% * $V_x$ and $V_y$ denote the longitudinal and lateral velocities of the
% vehicle in body-fixed coordinates with respect to the center of gravity
% $O$.
% * $\psi$ is the yaw angle of the vehicle.
% * $\delta$ is the front steering angle.
% * $Y_{ref}$ is the reference lateral position.
% * $\psi_{ref}$ is the reference yaw angle.
%
% The following vehicle parameter values are used in this example:
%
% * |m| is the total vehicle mass (kg)
% * |Iz| is the yaw moment of inertia of the vehicle (mNs^2).
% * |lf| is the longitudinal distance from the center of gravity to the
% front tires (m).
% * |lr| is the longitudinal distance from center of gravity to the rear
% tires (m).
% * |Cf| is the cornering stiffness of the front tires (N/rad).
% * |Cr| is the cornering stiffness of the rear tires (N/rad).
%
m = 1575;
I = 2875;
a = 1.2;
b = 1.6;
Cf = 19000;
Cr = 33000;

%%
% In this example, the longitudinal vehicle dynamics are separated from the
% lateral vehicle dynamics. Therefore, the longitudinal velocity is assumed
% to be constant. You can represent the lateral vehicle dynamics using a
% linear time-invariant (LTI) system with the following state, input, and
% output variables.
%
% * State variables: Lateral velocity $V_y$, yaw angle $\psi$,
% yaw angle rate $r$, global $Y$ position
% * Input variable: Front steering angle $\delta$
% * Output variable: Global $Y$ position and yaw angle $\psi$

%%
% Specify the longitudinal velocity in m/s.
Vx = 15;

%%
% Specify a state-space model, |vehicle|, of the lateral vehicle dynamics.
A = [0,     1,                      0,      0,                          0,      0;    
     0,     -(Cf+Cr)/(m*Vx),         0,      -Vx-(Cf*a-Cr*b)/(m*Vx),       0,      0; 
     0,     0,                      0,      1,                          0,      0;
     0,     -(Cf*a-Cr*b)/(I*Vx),     0,      -(Cf*a^2+Cr*b^2)/(I*Vx),     0,      0;
     0,     1,                      Vx,      0,                          0,      0
     0,     -1,                     Vx,      0,                          0,      0   ];
 
 
B = [0,     Cf/m,                   0,      Cf*a/I,                     0,      0   ]';

C = [0,     0,                      0,      0,                          1,      0;
     0,     0,                      1,      0,                          0,      0   ];

vehicle = ss(A,B,C,0);

%%
% This LTI model approximates the global $Y$ position of the vehicle using
% $\dot{Y} = V_y+V_x\psi$, which applies under small angle assumptions.

%% Reference Trajectory: Double Lane Change Maneuver
% The reference trajectory to be followed is generated for a double lane
% change maneuver scenario. Given the longitudinal velocity of the vehicle,
% $V_x$, define a reference trajectory for the vehicle using $Y_{ref}$ and
% $\psi_{ref}$. This scenario represents an emergency obstacle avoidance
% maneuver with a given longitudinal velocity. In this example, the
% obstacles are static and the reference trajectory that avoids the
% obstacles is predesigned.

%%
% Plot the reference path and obstacles.
T = 15;         % simulation duration
time = 0:0.1:T; % simulation time
plotReference(Vx,time);

%% Design Model Predictive Controller with Previewing
% In this example, the control objective is to avoid the obstacles by
% following the reference trajectory closely. The autonomous steering
% system is implemented using a linear MPC controller with look-ahead
% (previewing). This controller is similar to a driver steering model where
% the driver can look ahead and preview the path in front of the vehicle.
% With previewing, the controller takes actions in advance and thus
% improve tracking performance. In this example, the previewed reference
% trajectory contains two signals, position $Y_{ref}$ and yaw angle
% $\psi_{ref}$. The preview horizon is one second.
%
% Open Simulink model.
mdl = 'mpcVehicleSteering';
open_system(mdl)

%%
% To design a model predictive controller for the plant model, first create
% an MPC controller using a default prediction horizon (|10| steps),
% default control horizon (|2| moves), and sample time |Ts|. The preview
% horizon is the product of the sample time and the prediction horizon.
Ts = 0.1;
mpc1 = mpc(vehicle,Ts);

mpc1.PredictionHorizon = 10;
mpc1.ControlHorizon = 2;
mpc1.Optimizer.CustomCostFcn = true;

%%
% For this controller, there is one manipulated variable (front steering
% angle $\delta$) and there are two output variables ($Y$ position and yaw
% angle $\psi$).

%% 
% To ensure driver comfort, constrain the rate of change of the steering
% angle to |0.26| rad/s (approximately 15 deg/s).
mpc1.MV.RateMin = -0.26;
mpc1.MV.RateMax = 0.26;

%%
% To ensure tracking performance for both position $Y$ and yaw angle
% $\psi$, set both output tuning weights to be nonzero. Since the
% controller has one manipulated variable and two outputs, the system is
% underactuated. With nonzero weights for both outputs, perfect tracking is
% not expected. However, you can still guarantee satisfactory tracking
% performance by selecting appropriate weights.
mpc1.Weights.OV = [0.8 0.3];                            

%% Simulation Analysis
% Run the simulation with the designed model predictive controller. The
% simulation data is logged in the |logsout| data set.
mpc1.Model.Plant=minreal(mpc1.Model.Plant);
sim(mdl)

%%
% Plot the simulation result.
plotSimulation(logsout,Vx,time)

%%
% The vehicle trajectory follows the reference trajectory closely, thus
% avoiding the obstacles. When the vehicle travels along a straight line,
% that is when $V_y = 0$, both $Y_{ref}$ and $\psi_{ref}$ are tracked
% perfectly. This result is due to the relations $\dot{Y} = V_x\psi$ and
% $\dot{Y}_{ref} = V_x\psi_{ref}$. In other words, if either $Y = Y_{ref}$
% or $\psi = \psi_{ref}$ is satisfied, then the other is satisfied as well.

%%
% Remove example file folder from MATLAB path, and close Simulink model.
rmpath(fullfile(matlabroot,'examples','mpc_featured','main'));
bdclose(mdl)
