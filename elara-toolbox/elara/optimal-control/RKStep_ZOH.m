function x_k1 = RKStep_ZOH(F, x_k, u_k, h, A, b, c)
    %% Compute one integration step of an explicit Runge-Kutta method
    % with zero-order hold for the input u along the time step
    arguments
        % Function to integrate with arguments F(x,u)
        F       (1,1)

        % Values at current time step
        x_k     (:,1)

        % Input during the time step (constant)
        u_k     (:,1)

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
        x_i = x_k;
        for j = 1:i-1
            x_i = x_i + h*A(i,j)*K{j};
        end
        K{i} = F(x_i, u_k);
    end
    x_k1 = x_k + h * horzcat(K{:}) * b;  % weighted sum of stages
end