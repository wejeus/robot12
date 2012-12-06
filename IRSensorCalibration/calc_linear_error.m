%calculates the average error of the estimated linear function
% the linear function is decribed by parameters = [m, b] and k
% points are the sample points (2,n) with points(:,i) = [u_i;d_i]
function error = calc_linear_error(parameters, points)
error = 0;
n = size(points, 2);
for i=1:n 
    temp = parameters(1) * points(1,i) + parameters(2) - 1/(points(2,i) + parameters(3));
    error = error + temp * temp;
end
