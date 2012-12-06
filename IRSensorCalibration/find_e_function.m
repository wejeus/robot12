function parameters = find_e_function(points)
alpha = 2.0;
lambda = 2.6;
a = 0.2;
c = points(1,end);
parameters = [alpha; lambda; a; c];
parametersTemp = parameters;
prevError = calc_error(parameters, points);
error = prevError;

counter = 0;
while (prevError >= error && counter < 500)
    parameters = parametersTemp;
    prevError = error;
    gradient = calc_gradient(parameters, points);
    parametersTemp = parameters - 0.1 * gradient;
    error = calc_error(parametersTemp, points);
    counter = counter +1;
    str = sprintf(' step %i: error is %d',i,error);
    disp(str);
end    