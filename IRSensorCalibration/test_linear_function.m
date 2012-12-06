function [range,error] = test_linear_function(m,b,k,data)
    n = size(data,2);
    range = zeros(1,n);
    error = zeros(1,n);
    for i = 1:n
        range(i) = round((1/(m * data(1,i) + b) - k)*100)/100;
        error(i) = range(i) - data(2,i); 
    end