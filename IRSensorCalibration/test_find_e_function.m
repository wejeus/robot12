orig_parameters = [3.2;1.1;2;5];
points = create_sample_points(orig_parameters,100);

found_parameters = find_e_function(points);

pointsFound = create_sample_points(found_parameters, 100);

finalError = calc_error(found_parameters,points)

plot(points(2,:), points(1,:));
hold all;
plot(pointsFound(2,:),pointsFound(1,:));
