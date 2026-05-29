function x_k1 = RKStep(F, x_k, t, h, A, b, c)
    %% Compute one integration step of an explicit Runge-Kutta method
    arguments
        % Function to integrate with arguments F(t,x)
        F       (1,1) function_handle

        % Values at current time step
        x_k     (:,1)

        % Current time
        t       (1,1)

        % Time step
        h       (1,1)

        % Butcher tableau specifying the RK method
        A       (:,:) double
        b       (:,1) double
        c       (:,1) double
    end
    s = length(b);             % number of stages
    K = cell(s,1);
    for i = 1:s
        t_i = t + c(i)*h;
        x_i = x_k;
        for j = 1:i-1
            x_i = x_i + h*A(i,j)*K{j};
        end
        K{i} = F(t_i, x_i);
    end
    x_k1 = x_k + h * horzcat(K{:}) * b;  % weighted sum of stages
end