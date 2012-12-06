function blub = get_blub(data,k)
n = size(data,2);
blub = zeros(1,n);
for i=1:n
    blub(i) = 1/(data(i) + k);
end