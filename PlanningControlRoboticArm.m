%%%%%%%%%%%%%%%%%%%%%% SAMPLE MODEL %%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%

% Trajectory Planning
% The robot initially rests at |[-10,-10]| with an orientation angle of
% |pi/2| radians (facing north). The flying maneuver for this example is to
% move and park the robot at the final location |[0,0]| with an angle of
% |0| radians (facing east) in |12| seconds. The goal is to find the
% optimal path such that the total amount of fuel consumed by the thrusters
% during the maneuver is minimized.
%
% Nonlinear MPC is an ideal tool for trajectory planning problems because
% it solves an open-loop constrained nonlinear optimization problem given
% the current plant states. With the availability of a nonlinear dynamic
% model, MPC can make more accurate decisions.
%
% Create a nonlinear MPC object with |6| states, |6| outputs, and |4|
% inputs. By default, all the inputs are manipulated variables (MVs).
nx = 6;
ny = 6;
nu = 3;
nlobj = nlmpc(nx,ny,nu);

%%
% For this example, the target prediction time is |12| seconds. Therefore,
% specify a sample time of |0.4| seconds and prediction horizon of |30|
% steps.
Ts = 0.4;

%%
% Each thrust has an operating range between |0| and |1|, which is
% translated into lower and upper bounds on the MVs.
for ct = 1:nu
    nlobj.MV(ct).Min = 0;
    nlobj.MV(ct).Max = 1;
end

%%
% Specify the initial conditions for the robot.
x0 = [-10;-10;pi/2;0;0;0];  % robot parks at [-10, -10], facing north
u0 = zeros(nu,1);           % thrust is zero

%% Feedback Control for Path Following
% After the optimal trajectory is found, a feedback controller is required
% to move the robot along the path. In theory, you can apply the optimal MV
% profile directly to the thrusters to implement feed-forward control.
% However, in practice, a feedback controller is needed to reject
% disturbances and compensate for modeling errors.
%
% You can use different feedback control techniques for tracking. In this
% example, you use another nonlinear MPC controller to move the robot to
% the final location. In this path tracking problem, you track references
% for all six states.
nlobj_tracking = nlmpc(nx,ny,nu);

%% 
% Use the same state function and its Jacobian function.
nlobj_tracking.Model.StateFcn = "RoboticArmStateFcn";
nlobj_tracking.Jacobian.StateFcn = @RoboticArmStateJacobianFcn;

%%
% For feedback control applications, reduce the computational effort by
% specifying shorter prediction and control horizons.
nlobj_tracking.Ts = Ts;
nlobj_tracking.PredictionHorizon = 10;
nlobj_tracking.ControlHorizon = 4;

%% 
% The default cost function in nonlinear MPC is a standard quadratic cost
% function suitable for reference tracking and disturbance rejection. For
% tracking, the states have higher priority (larger penalty weights) than
% the MV moves.
nlobj_tracking.Weights.ManipulatedVariablesRate = 0.2*ones(1,nu);
nlobj_tracking.Weights.OutputVariables = 5*ones(1,nx);

%% 
% Set the same bounds for the thruster inputs.
for ct = 1:nu
    nlobj_tracking.MV(ct).Min = 0;
    nlobj_tracking.MV(ct).Max = 1;
end

%%
% Also, to reduce fuel consumption, it is clear that |u1| and |u2| cannot be
% positive at any time during the operation. Therefore, implement equality
% constraints such that |u(1)*u(2)| must be |0| for all prediction steps.
% Apply similar constraints for |u3| and |u4|.
nlobj_tracking.Optimization.CustomEqConFcn = ...
    @(X,U,data) [U(1:end-1,1).*U(1:end-1,2); U(1:end-1,3).*U(1:end-1,4)];

%%
% Validate the your prediction model and custom functions, and their Jacobians.
validateFcns(nlobj_tracking,x0,u0);

%% Nonlinear State Estimation
% In this example, only the three position states (x, y and angle) are
% measured. The velocity states are unmeasured and must be estimated. Use
% an extended Kalman filter (EKF) from Control System Toolbox(TM) for
% nonlinear state estimation.
%
% Because an EKF requires a discrete-time model, you use the trapezoidal
% rule to transition from x(k) to x(k+1), which requires the solution of
% |nx| nonlinear algebraic equations. For more information, open
% |FlyingRobotStateFcnDiscreteTime.m|.
DStateFcn = @(xk,uk,Ts) RoboticArmStateFcnDiscreteTime(xk,uk,Ts);

%%
% Measurement can help the EKF correct its state estimation. Only the first
% three states are measured.
DMeasFcn = @(xk) xk(1:3);

%%
% Create the EKF, and indicate that the measurements have little noise.
EKF = extendedKalmanFilter(DStateFcn,DMeasFcn,x0);
EKF.MeasurementNoise = 0.01;

mdl = 'mpc_pendcartNMPC';
open_system(mdl)