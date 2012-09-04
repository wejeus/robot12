function Theta = K_i(x, y, z)

L_1 = .4;
L_2 = .5;
d_1 = .2;


Theta = [ acos(x / sqrt(x^2 + y^2)) - acos((L_1^2 + x^2 + y^2 - L_2^2) / (2*L_1*sqrt(x^2 + y^2)));
          acos((x^2 + y^2 - L_1^2 - L_2^2) / (2 * L_1 * L_2));
          d_1 - z ];
      
% Theta = [ acos(x / sqrt(x^2 + y^2)) - acos((L_1^2 + x^2 + y^2 - L_2^2) / (2*L_1*sqrt(x^2 + y^2)));
%           acos((x^2 + y^2 - L_1^2 - L_2^2) / (2 * L_1 * L_2));
%           d_1 - z ];
% if Theta(2) < pi/4 && Theta(2) > -pi/4
%     Theta(4) = acos(x / sqrt(x^2 + y^2)) + acos((L_1^2 + x^2 + y^2 - L_2^2) / (2*L_1*sqrt(x^2 + y^2)));
%     Theta(5) = -acos((x^2 + y^2 - L_1^2 - L_2^2) / (2 * L_1 * L_2));
%     Theta(6) = d_1 - z;
% end
