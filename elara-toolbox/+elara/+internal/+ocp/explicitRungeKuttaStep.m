function x_k1 = explicitRungeKuttaStep(F, x_k, u_k, h, A, b, c)
    %% Compute one integration step of an explicit Runge-Kutta method
    arguments
        % Function to integrate with arguments F(x,u)
        F       (1,1)

        % Values at current time step
        x_k     (:,1)

        % Matrix of input values at the RK stages,
        % dimensions (nInputs, s)
        u_k     (:,:)

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
        K{i} = F(x_i, u_k(:,i));
    end
    x_k1 = x_k + h * horzcat(K{:}) * b;  % weighted sum of stages
end
