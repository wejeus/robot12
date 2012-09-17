function J = jacobian(Theta)

global L_1
global L_2

J = [ -L_1 * sin(Theta(1)) - L_2 * sin(Theta(1) + Theta(2)), -L_2 * sin(Theta(1) + Theta(2)), 0;
       L_1 * cos(Theta(1)) + L_2 * cos(Theta(1) + Theta(2)),  L_2 * cos(Theta(1) + Theta(2)), 0;
       0                                                   ,  0                             , -1
];
