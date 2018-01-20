%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
% Trajectory Generation for Moving Ground Targets in SE(2)
%
% Scenario/Assumptions:
%   - Targets moving in a circle
%   - Targets moving in a square
%   - Targets moving in figure-8 (lemniscate)
%
% See `sl_pursuit.slx` in Peter Corke's Robotics Toolbox. See Sec 4.1.2 of
% Robotics, Vision and Control (Corke) for discussion of unicycle model.
%
% Parker Lusk, BYU
% 17 January 2018
%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
clear all, clc;

% =========================================================================
% === Environment Setup ===================================================
% =========================================================================

% Simulation Timing
Ts = 0.1;       % Sample rate

% create a path for the robot to follow
waypoints = [5 5; 10 8; 7 0; 15 -3; 15 4; -2 4];
vel = [1 1];
[robotpath, Tf] = trajWaypoints(waypoints, vel, Ts);

% % generate a circular trajectory
% r = 3;
% C = 2*pi*r;
% Tf = C / 1;
% ang = 0:Ts:8*pi; 
% robotpath = r*[cos(ang); sin(ang)]' + [2 5];

% % generate lemniscate trajectory
% ang = linspace(-pi/4,pi/4);
% a = 3;
% x = a*cos(ang).*sqrt(2*cos(2.*ang));
% y = a*sin(ang).*sqrt(2*cos(2.*ang));
% robotpath = [x -fliplr(x); y -fliplr(y)]' + [5 5];
% Tf = 20;

% Robot parameters
kp_heading = 4;

% Draw the environment
figure(1), clf;
plot(robotpath(:,1), robotpath(:,2), 'r--', 'LineWidth', 2);
scatter(robotpath(:,1), robotpath(:,2), 20, 'ro');
xaxis([min(waypoints(:,1))-2 max(waypoints(:,1))+2]);
yaxis([min(waypoints(:,2))-2 max(waypoints(:,2))+2]);
grid on; hold on; axis equal;

% Initial conditions
x = [0 0 0]';

% Setup drawing handles
bot = drawRobot(x, 1, []);
goalHandle = scatter([], [], 40, 'k*');

% =========================================================================
% === Main Event Loop =====================================================
% =========================================================================

% vector of timesteps
tvec = 0:Ts:Tf;

% Keep track of variables for plotting
UU = zeros(length(tvec), 2);
XX = zeros(length(tvec), 3);

for i = 1:length(tvec)
    t = tvec(i);
    
    % Select the current goal position from the trajectory -- think moving
    % carrot.
    goal = getCurrentGoalFromTrajectory(robotpath, t, Ts, Tf);
    set(goalHandle, 'XData', goal(1), 'YData', goal(2));
    
    % Generate the robot commands
    v = speedControl(x, goal, t, Ts);
    w = headingControl(x, goal, kp_heading);
    
    % Move the robot
    u = [v w]';
    x = unicycle(x, u, Ts);
    bot = drawRobot(x, 0, bot);
    
    % save for later
    UU(i,:) = u';
    XX(i,:) = x';

    % Animate
    drawnow;
end

figure(2), clf;
subplot(511); plot(tvec, XX(:,1)); ylabel('x'); grid on; hold on;
% plot(tvec, [robotpath(:,1); robotpath(end,1)], 'r--','LineWidth',2);
subplot(512); plot(tvec, XX(:,2)); ylabel('y'); grid on; hold on;
% plot(tvec, [robotpath(:,2); robotpath(end,2)], 'r--','LineWidth',2);
subplot(513); plot(tvec, rad2deg(XX(:,3))); ylabel('\theta'); grid on;
subplot(514); plot(tvec, UU(:,1)); ylabel('v'); grid on; hold on;
plot(tvec, min(vel)*ones(size(tvec)), 'k--');
plot(tvec, max(vel)*ones(size(tvec)), 'k--');
subplot(515); plot(tvec, UU(:,2)); ylabel('\omega'); grid on;
xlabel('seconds');


% =========================================================================
% ==== Helper Functions ===================================================
% =========================================================================

function v = speedControl(robot, goal, t, Ts)
%SPEEDCONTROL Generate the speed cmd necessary to drive to goal]
%
%   v = speedControl(robot, goal) finds the difference between the robot's
%   current position and the goal in R^2 and generates a speed command v.

% Calculate error to goal
e = goal - robot(1:2);

% Allow a "look-ahead" distance in the distance error. The purpose of this
% is to generate smoother commands
d = 1;
de = norm(e) - d;

% Implement PID control to regulate the distance error to zero.
v = simplePID(de, 0.5, 0.1, 0, 5, Ts, 0.05, t==0);
end

% -------------------------------------------------------------------------

function w = headingControl(robot, goal, kp)
%HEADINGCONTROL Generate the heading rate cmd necessary to drive to goal

% Calculate error to goal
e = goal - robot(1:2);

% Calculate heading to the goal
gamma = atan2(e(2), e(1));

% Calculate heading error
he = gamma - robot(3);
% he = angdiff(gamma, robot(3));

% Angle wrapping to keep \in [-pi, pi]
% if      he >  pi, he = he - 2*pi;
% elseif  he < -pi, he = he + 2*pi;
% end

while he >  pi, he = he - 2*pi; end
while he < -pi, he = he + 2*pi; end

% he = mod(he+pi, 2*pi) - pi;

% Generate the heading rate command
w = he*kp;
end

% -------------------------------------------------------------------------

function u = simplePID(error, kp, ki, kd, limit, Ts, sigma, flag)
%SIMPLEPID Implements a simple PID controller

persistent integrator;
persistent differentiator;
persistent error_d1;

% If first iteration, initialize persistent variables
if flag == 1
    integrator      = 0;
    differentiator  = 0;
    error_d1        = 0;
end

% Update differentiator
a1 = (2*sigma - Ts) / (2*sigma + Ts);
a2 = 2/(2*sigma + Ts);
differentiator = a1*differentiator + a2*(error-error_d1);

% Update integrator
integrator = integrator + (Ts/2)*(error + error_d1);

% Implement PID controller
u_unsat = kp*error + ki*integrator + kd*differentiator;

% Saturate command signal
if      u_unsat >  limit, u = limit;
elseif  u_unsat < -limit, u = -limit;
else                    , u = u_unsat;
end

% Integrator anti-windup
if ki~=0
    integrator = integrator + (Ts/ki)*(u - u_unsat);
end

% Save error for next time
error_d1 = error;
end

% -------------------------------------------------------------------------

function [robotpath, tseg] = trajWaypoints(waypoints, vel, Ts)
%TRAJWAYPOIONTS Generate a straight-line trajectory through given waypoints
%
%   x = trajLine(x0, x1) creates a straight-line trajectory x in R^2
%   starting at the point x0 and ending at the point x1.

robotpath = my_mstraj(waypoints, vel, Ts);

tseg = numrows(robotpath)*Ts;
end

% -------------------------------------------------------------------------

function traj = my_mstraj(segments, qdmax, dt)
%MY_MSTRAJ Multi-segment multi-axis trajectory
%
% A trajectory is a path with specified timing
%

ns = numrows(segments); % num of segments
nj = numcols(segments); % num of axis/dimensions

% initial conditions
q_prev = segments(1,:);
qd_prev = zeros(1, nj);

clock = 0;      % keep track of time
arrive = [];    % record planned time of arrival at each waypoint

traj = [];      % keep track of points in the trajectory

for seg = 2:ns
    % Setup next target waypoint
    q_next = segments(seg,:); % current target
    q_diff = q_next - q_prev; % total distance to move this segment
    
    %
    % qdmax is specified, compute slowest dimension
    %
    
    % how long will it take to travel this segment?
    tl = abs(q_diff) ./ qdmax;
    tl = ceil(tl/dt)*dt;    % ensure divisibility by sample rate
    
    % find the total time and slowest dimension
    [tseg, slowest] = max(tl);
    
    % log the planned arrival time
    arrive(seg) = clock + tseg;
    
    %
    % create the trajectory for this segment
    %
    
    % linear velocity from q_prev to q_next
    qd = q_diff / tseg;
    
    for t = dt:dt:tseg
        s = t/tseg;
        q = (1-s)*q_prev + s*q_next;
        traj = [traj; q];
        clock = clock + dt;
    end
    
    % The current target (q_next) is now behind us.
    q_prev = q_next;
    qd_prev = qd;
end

end

% -------------------------------------------------------------------------

function goal = getCurrentGoalFromTrajectory(robotpath, t, dt, Tf)
%GETCURRENTGOALFROMTRAJECTORY Select the current goal of a trajectory
%
%   goal = getCurrentGoalFromTrajectory(robotpath, t, Tf) returns the
%   current goal from the robotpath based on the time, t.

% goal = interp1(linspace(0,Tf,numrows(robotpath)), robotpath, t)';

% Which point in the robot path should we be choosing?
i = ceil(t / dt) + 1;

if i > numrows(robotpath)
    goal = robotpath(end,:)';
else
    goal = robotpath(i,:)';
end

end

% -------------------------------------------------------------------------

function x = unicycle( x, u, dt )
%UNICYCLE Simple, noisless unicycle (differential-drive) ground robot model
%   See Sec 4.1.2 (p. 109) of RVC, Peter Corke
%
%   INPUTS:
%   x = [x y theta]^T   current state
%   u = [v w]^T         (constant speed during dt)
%   dt = timestep (needs to small, e.g., 0.01)
%
%   OUTPUTS:
%   x = [x y theta]^T   next state

% Break out last state for convenience
theta = x(3);

% Break out control inputs for convenience
v = u(1); % linear velocity -- in the direction of the heading
w = u(2); % angular velocity -- about some circle with r = v/w

% Calculate resulting state eq (5.16)
x = x + [
            v*dt*cos(theta)
            v*dt*sin(theta)
            w*dt
        ];
end

% -------------------------------------------------------------------------

function handle = drawRobot( xt, animate, handle )
%DRAWROBOT Draws a SE(2) robot
%   inputs:
%       xt: input state vector (x, y, theta)
%   outputs:
%       handle: the figure handle for the robot

    if nargin == 2 || isempty(handle)
        handle_circle = [];
        handle_line = [];
        handle_path = [];
    else
        % Unpack the handles
        handle_circle = handle(1);
        handle_line = handle(2);
        handle_path = handle(3);
    end

    r = 0.5; % this comes from how we did the meshgrid when plotting with
             % `pcolor`. Since we start in the cetner of the square cell of
             % width 1, radius must be 0.5
    
    % Unpack the robot's states
    x = xt(1);
    y = xt(2);
    theta = xt(3);
            
    handle_circle = circle(x, y, r, handle_circle);
    handle_line = heading(x, y, r, theta, handle_line);
    handle_path = path(x, y, handle_path);
    
    if animate == 1
        drawnow; % draw the entire robot together
    end
    
    handle = [handle_circle handle_line handle_path];
end

% -------------------------------------------------------------------------

function handle = circle(x,y,r, handle)
%x and y are the coordinates of the center of the circle
%r is the radius of the circle
%0.01 is the angle step, bigger values will draw the circle faster but
%you might notice imperfections (not very smooth)
    ang = 0:0.01:2*pi; 
    xp = r*cos(ang);
    yp = r*sin(ang);
    
    if isempty(handle)
        handle = plot(x+xp,y+yp, 'Color', [0 0.4470 0.7410]);
    else
        set(handle,'XData',x+xp,'YData',y+yp);
    end
end

% -------------------------------------------------------------------------

function handle = heading(x,y,r,theta, handle)
    x2 = x + r*cos(theta);
    y2 = y + r*sin(theta);
    
    if isempty(handle)
        handle = plot([x x2],[y y2], 'Color', [0.8500 0.3250 0.0980]);
    else
        set(handle,'XData',[x x2],'YData',[y y2]);
    end
end

% -------------------------------------------------------------------------

function handle = path(x,y,handle)
    if isempty(handle)
        handle = plot(x, y, 'k');
    else
        XX = [get(handle, 'XData') x];
        YY = [get(handle, 'YData') y];
        set(handle,'XData',XX,'YData',YY);
    end
end