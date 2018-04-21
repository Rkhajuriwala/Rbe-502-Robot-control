function [f,dfdy,dfdu,dfddu,dfdslack] = mpcCustomCostFcn(y,yref,u,uref,du,v,slack,varargin)
% This is a template for specifying arbitrary cost "f" in MPC controller.  
% This cost function replaces the standard quadratic cost function.
% "fmincon" from Optimization Toolbox is used to solve the nonlinear programming problem.
% Set "Optimizer.CustomCostFcn = true" in MPC controller to enable this feature.
% Gradients are optional although having them would help with computational efficiency.
%
% Inputs:
%   y:      predicted plant outputs, p-by-ny, from k+1 to k+p 
%   yref:   previewed output reference, p-by-ny, from k+1 to k+p
%   u:      optimal control sequence, p-by-nmv, from k to k+p-1
%   uref:   previewed mv target, p-by-nmv, from k to k+p-1 (although we don’t provide preview for mv target now, this interface is general)
%   du:     rate of control changes, p-by-nmv, from k to k+p-1 (the first value is u(k)-u(k-1))
%   v:      previewed MD, p+1-by-nmd, from k to k+p ([] is passed in if MD does not exist)
%   slack:  global slack variable, scalar
%
% Outputs:
%           f:    nonlinear cost, scalar
%
%        dfdy:    df/dy, p-by-ny, (return [] if unknown)
%        dfdu:    df/du, p-by-nmv, (return [] if unknown)
%       dfddu:    df/ddu, p-by-nmv, (return [] if unknown)
%    dfdslack:    df/dslack, 1-by-1, (return [] if unknown)
%
% Note that:
%   (1) Prediction horizon can be obtained as "size(y,1)".
%   (3) The previous MV, u(k-1), can be computed as "u(1,:)-du(1,:)".
%   (3) Economic MPC does not support "block moves".
%
% In the template, standard quadratic cost function with default weights is implemented as an example.

% Dimension
p = size(y,1);
nmv = size(u,2);
ny = size(y,2);
% Default weights (MV weights are 0)
Wy = 4*diag(ones(ny,1));
Wdu = diag(ones(nmv,1))*0.1; 
Wecr = 1e5;
% Quadratic cost
f = sum(sum(((y-yref)*Wy).^2))+sum(sum((du*Wdu).^2))+Wecr*slack^2;
% Gradients
dfdy = (y-yref)*(Wy.^2);
dfdu = zeros(p,nmv);
dfddu = du*(Wdu.^2);
dfdslack = Wecr*slack;
