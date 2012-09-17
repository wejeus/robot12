function result = valid_configuration(Theta, start_pos, v)

v_normal = (cross(v / norm(v), [0 0 1]));

result = abs((K_f(Theta(1),Theta(2),Theta(3)) - start_pos') * v_normal) < 0.01;