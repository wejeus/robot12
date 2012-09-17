clear, clc, close all
% set parameters

global L_1
global L_2
global d_1

L_1 = .4;
L_2 = .4;
d_1 = .2;

theta_1 = pi/5;
theta_2 = pi/5;
L_3 = .1;



% calc cartesian coordinates
X_cart = K_f(theta_1, theta_2, L_3);

% calc joint space coordinates
Theta = K_i(X_cart(1), X_cart(2), X_cart(3));

% 
% if size(Theta,1) > 3
%     X_cart2 = K_f(Theta(4), Theta(5), Theta(6))
% 
%     norm(X_cart2 - X_cart) < 0.0001
%     
% end

% compare resut to original to see if K_f and K_i works
abs(theta_1 - Theta(1)) < 0.0001;
abs(theta_2 - Theta(2)) < 0.0001;
abs(L_3 - Theta(3))     < 0.0001;

% Velocity Jacobian
J = [ -L_1 * sin(theta_1) - L_2 * sin(theta_1 + theta_2), -L_2 * sin(theta_1 + theta_2), 0;
       L_1 * cos(theta_1) + L_2 * cos(theta_1 + theta_2), L_2 * cos(theta_1 + theta_2) , 0;
       0                                                , 0                            , -1
];

% ------------------
% --- Simulation ---
% ------------------

%% ---- set star/end points ----
start_point = [0.1 0.3 0];
end_point   = [-0.5 0.1 0];
Theta_start = K_i(start_point(1), start_point(2), start_point(3));
Theta_end   = K_i(end_point(1), end_point(2), end_point(3));

v = (end_point - start_point) / 50;

% ---- move to start pos ---- 
Theta_i = Theta_start;
d1      = 0;
margin                  = 15; % tune this to change trajectory from init to start.
stepsToLowerManipulator = 21;

for j=1:stepsToLowerManipulator + margin
    
    trajectory(1,j) = 180/pi * Theta_i(1);
    trajectory(2,j) = 180/pi * Theta_i(2);
    trajectory(3,j) = d1;
    
    % increase d1 when there is 20 steps left to start pos
    if stepsToLowerManipulator + margin - j < stepsToLowerManipulator
        d1 = d1 + 0.01;
    end
end

% ---- calc optimal speed ---- 
v_norm = v / norm(v);
v_min  = inf;
Theta_old = Theta_start;

% calculate a speed limit on the path by calculating a trajectory with a
% low velocity first
while(norm(end_point - K_f(Theta_i(1), Theta_i(2), Theta_i(3))') > 0.01)
    J_inv = inv(jacobian(Theta_i));
    Theta_i = Theta_i + J_inv * v' .* 0.1;
    speed = J_inv * v_norm';
    
    next = min((pi/180)*50/abs(speed(1)), (pi/180)*50/abs(speed(2)));
    v_min = min(next, v_min);
end

% v_min is now the maximal speed that is approximately allowed on this
% path.

% To end up at end_point we have to adapt this speed first.
% We can end up at the end_position if there is an integer number of steps
% so that end_point = start_point + steps * v_min * v
length = norm(start_point - end_point); % length of the complete line
steps = ceil (length / (v_min * 0.1)); % number of steps (sample points)
v_min = length / steps / 0.1; % new velocity that allows us to end up close to the end_point

% set speed
v = v_min * v_norm;
v_orig = v;


%% ---- generate trajectory with "optimal" speed ----
i = stepsToLowerManipulator + margin + 1;
Theta_i   = Theta_start;
Theta_old = Theta_i;

% set start configuration (theta) first
trajectory(1,i) = 180/pi * Theta_i(1);
trajectory(2,i) = 180/pi * Theta_i(2);
trajectory(3,i) = Theta_i(3);
i = i + 1;

while(norm(end_point - K_f(Theta_i(1), Theta_i(2), Theta_i(3))') > 0.005)
    J_inv = inv(jacobian(Theta_i));
    Theta_old = Theta_i;
    Theta_i = Theta_i + J_inv * v' .* 0.1;
    
    trajectory(1,i) = 180/pi * Theta_i(1);
    trajectory(2,i) = 180/pi * Theta_i(2);
    trajectory(3,i) = Theta_i(3);
    i = i + 1;
    
    % correct error
    currentPosOnLine = K_f(Theta_i(1), Theta_i(2), Theta_i(3))' - start_point;
    v_normal         = cross(v_orig / norm(v_orig), [0 0 1]);
    error            = currentPosOnLine * v_normal';
    v                = v_orig - 10 * error * v_normal;
    v                = v / norm(v) * v_min;
end

% ---- Simulate trajectory ----
X_cart = rob_sim(trajectory',2);

% ---- Check weld ----
 [success time mean_vel std_vel max_dev] = check_weld(X_cart)


% ---- Plot trajectory (x,y,z & theta_1,theta_2,L_3) ----
figure()
time = 0:0.1:length(X_cart)*0.1-0.1;

% plot x, y, z
subplot(6,2,1)
hold on
xlabel('Time (s)')
ylabel('X-coordinate (m)')
plot(time, X_cart(:,1)')

subplot(6,2,3)
hold on
xlabel('Time (s)')
ylabel('Y-coordinate (m)')
plot(time, X_cart(:,2)')

subplot(6,2,5)
hold on
xlabel('Time (s)')
ylabel('Z-coordinate (m)')
plot(time, X_cart(:,3)')

% plot theta_1, theta_2, L_3
subplot(6,2,2)
hold on
xlabel('Time (s)')
ylabel('Theta_1-coordinate (degrees)')
plot(time(1:length(trajectory)), trajectory(1,:))

subplot(6,2,4)
hold on
xlabel('Time (s)')
ylabel('Theta_2-coordinate (degrees)')
plot(time(1:length(trajectory)), trajectory(2,:))

subplot(6,2,6)
hold on
xlabel('Time (s)')
ylabel('L3-coordinate (m)')
plot(time(1:length(trajectory)), trajectory(3,:))
