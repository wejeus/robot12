theta_1 = pi/5
theta_2 = pi/5
L_1 = .4;
L_2 = .5;
L_3     = .1

X_cart = K_f(theta_1, theta_2, L_3)
Theta = K_i(X_cart(1), X_cart(2), X_cart(3))

% if size(Theta,1) > 3
%     X_cart2 = K_f(Theta(4), Theta(5), Theta(6))
% 
%     norm(X_cart2 - X_cart) < 0.0001
%     
% end

abs(theta_1 - Theta(1)) < 0.0001
abs(theta_2 - Theta(2)) < 0.0001
abs(L_3 - Theta(3))     < 0.0001

J = [ -L_1 * sin(theta_1) - L_2 * sin(theta_1 + theta_2), -L_2 * sin(theta_1 + theta_2), 0;
      -L_1 * cos(theta_1) - L_2 * cos(theta_1 + theta_2), -L_2 * cos(theta_1 + theta_2), 0;
      0                                                  , 0                            , -1
];
