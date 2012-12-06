function gradient = calc_mbk_gradient(parameters, data)

n = size(data, 2);
m = parameters(1);
b = parameters(2);
k = parameters(3);

dM = 0;
for i = 1:n
    dM = dM + data(1,i) * (m * data(1,i) + b - 1/(data(2,i) + k));
end

dB = 0;
for i = 1:n
    dB = dB + m * data(1,i) + b - 1/(data(2,i) + k);
end

dK = 0;
for i = 1:n
    dK = dK + 1/(data(2,i) + k) * m * data(1,i) + b - 1/(data(2,i) + k);
end

gradient = [dM; dB; dK];
gradient = 1/norm(gradient)*gradient;