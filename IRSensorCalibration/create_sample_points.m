function points = create_sample_points(parameters, n)

points = zeros(2,n);
for i = 1:n
    d = (i-1) / 5;
    points(1,i) = parameters(1) * exp(-parameters(2) * d + parameters(3)) + parameters(4);
    points(2,i) = d;
end