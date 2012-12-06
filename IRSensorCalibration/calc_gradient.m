function gradient = calc_gradient(parameters, points)

n = size(points, 2);

dAlpha = 0;

for i = 1:n
    e_stuff = exp(-parameters(2) * points(2,i) + parameters(3));
    dAlpha = dAlpha + e_stuff * (parameters(1) * e_stuff + parameters(4) - points(1,i));
end

dLambda = 0;
for i = 1:n
    e_stuff = exp(-parameters(2) * points(2,i) + parameters(3));
    dLambda = dLambda - parameters(1) * points(2,i) * e_stuff * (parameters(1) * e_stuff + parameters(4) - points(1,i));
end

dA = 0;
for i = 1:n
    e_stuff = exp(-parameters(2) * points(2,i) + parameters(3));
    dA = dA + parameters(1) * e_stuff * (parameters(1) * e_stuff  + parameters(4) - points(1,i));
end

dC = 0;
%for i = 1:n
 %   dC = dC + parameters(1) * exp(-parameters(2) * points(2,i) + parameters(3)) + parameters(4) - points(1,i);
%end

gradient = [dAlpha; dLambda; dA; dC];
gradient = 1/norm(gradient)*gradient;