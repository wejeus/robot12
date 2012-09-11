
% set parameters
theta_1 = pi/5
theta_2 = pi/5
L_1 = .4;
L_2 = .5;
L_3     = .1

%calc cartesian coordinates
X_cart = K_f(theta_1, theta_2, L_3)

% calc joint space coordinates
Theta = K_i(X_cart(1), X_cart(2), X_cart(3))

% 
% if size(Theta,1) > 3
%     X_cart2 = K_f(Theta(4), Theta(5), Theta(6))
% 
%     norm(X_cart2 - X_cart) < 0.0001
%     
% end

% compare resut to original to see if K_f and K_i works
abs(theta_1 - Theta(1)) < 0.0001
abs(theta_2 - Theta(2)) < 0.0001
abs(L_3 - Theta(3))     < 0.0001

% Velocity Jacobian
J = [ -L_1 * sin(theta_1) - L_2 * sin(theta_1 + theta_2), -L_2 * sin(theta_1 + theta_2), 0;
      -L_1 * cos(theta_1) - L_2 * cos(theta_1 + theta_2), -L_2 * cos(theta_1 + theta_2), 0;
      0                                                  , 0                            , -1
];

% ------------------
% --- Simulation ---
% ------------------

% create trajectory from [0.1 0.3 0] to [-0.5 0.1 0]
start_point = [0.1 0.3 0];
end_point   = [-0.5 0.1 0];
X_cart = 

% (always start at 60 -40 0.1, robot arm is starting at that position)
% trajectory = [60 -40 0.1;
%              ];
          
% simulate trajectory with rob_sim
draw_robot = 1;  % 0=don't draw, 1= , 2=
X_cart = rob_sim(trajectory, draw_robot)