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



%calc cartesian coordinates
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

% create trajectory from [0.1 0.3 0] to [-0.5 0.1 0]

%%
start_point = [0.1 0.3 0];
end_point   = [-0.5 0.1 0];

v = (end_point - start_point) / 50;
Theta_start = K_i(start_point(1), start_point(2), start_point(3));
Theta_end   = K_i(end_point(1), end_point(2), end_point(3));
trajectory  = Theta_start;
i=103;
Theta_i = Theta_start;

t=[];
for j=1:100
    trajectory(1,j) = 180/pi * Theta_i(1);
    trajectory(2,j) = 180/pi * Theta_i(2);
    trajectory(3,j) = 0;
end

%%

while(norm(end_point - K_f(Theta_i(1), Theta_i(2), Theta_i(3))') > 0.01)
    J_inv = inv(jacobian(Theta_i));
    Theta_i = Theta_i + J_inv * v' .* 0.1;
    trajectory(1,i) = 180/pi * Theta_i(1);
    trajectory(2,i) = 180/pi * Theta_i(2);
    trajectory(3,i) = Theta_i(3);
    i=i+1;
end

trajectory(1,102) = 180/pi * Theta_start(1);
trajectory(2,102) = 180/pi * Theta_start(2);
trajectory(3,102) = Theta_start(3);

X_cart = rob_sim(trajectory',2)
