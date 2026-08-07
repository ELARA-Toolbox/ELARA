function x_k1 = explicitRungeKuttaStepLinearInput(F, x_k, u_k, u_k1, h, A, b, c)
    %% Compute one integration step of an explicit Runge-Kutta method
    % with linear interpolation of the inputs (from the node values at k
    % and k+1)
    arguments
        % Function to integrate with arguments F(x,u)
        F       (1,1)

        % Values at current time step
        x_k     (:,1)

        % Input at time steps k and k+1
        u_k     (:,1)
        u_k1    (:,1)

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
        u_i = (1 - c(i)) * u_k + c(i) * u_k1;
        for j = 1:i-1
            x_i = x_i + h*A(i,j)*K{j};
        end
        K{i} = F(x_i, u_i);
    end
    x_k1 = x_k + h * horzcat(K{:}) * b;  % weighted sum of stages
end
