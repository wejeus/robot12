function parameters = find_linear_function(data)
k = 0.052;

counter = 0;

% calculate error for k
p = polyfit(data(1,:),get_blub(data(2,:),k),1);
parameters = [p(1);p(2);k];
error = calc_linear_error(parameters,data);

while (counter < 100)    
    
    k_step = 2^(-ceil(counter/3) - 5);
    
    % calculate error for k_left
    k_left = k - k_step;
    p_left = polyfit(data(1,:),get_blub(data(2,:),k_left),1);
    parameters = [p_left(1);p_left(2);k_left];
    error_left = calc_linear_error(parameters,data);
      
    % calculate error for k_right
    k_right = k + k_step;
    p_right = polyfit(data(1,:),get_blub(data(2,:),k_right),1);
    parameters = [p_right(1);p_right(2);k_right];
    error_right = calc_linear_error(parameters,data);
    
    if (error_left < error && error_left < error_right) 
        k = k_left;
        parameters = [p_left(1);p_left(2);k];
        error = error_left;
    elseif (error_right < error && error_right < error_left)
        k = k_right;
        parameters = [p_right(1);p_right(2);k];
        error = error_right;
    else
        counter = counter + mod(counter,3);
    end
    counter = counter +1;
    %str = sprintf(' step %i: k is %d and error is %d',counter,k,error);
    %disp(str);
end    