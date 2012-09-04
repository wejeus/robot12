function X_cart = K_f(theta_1, theta_2, L_3)

L_1 = .4;
L_2 = .5;
d_1 = .2;

X_cart = [ L_1 * cos(theta_1) + L_2 * cos(theta_1 + theta_2);
           L_1 * sin(theta_1) + L_2 * sin(theta_1 + theta_2);  
           d_1 - L_3 ];

end


