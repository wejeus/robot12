%calculates the error of the estimated exponential function
% the e function is decribed by parameters = [alpha, lambda, a, c]
% points are the sample points (2,n) with points(:,i) = [u_i;d_i]
function error = calc_error(parameters, points)
error = 0;
n = size(points, 2);
for i=1:n 
    temp = parameters(1) * exp(-parameters(2) * points(2,i) + parameters(3)) + parameters(4) - points(1,i);
    error = error + temp * temp;
end
error = 0.5 * error;