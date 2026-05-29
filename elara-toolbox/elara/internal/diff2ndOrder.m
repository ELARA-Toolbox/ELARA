function [x_dot, x_ddot] = diff2ndOrder(x, h)
    %% Compute derivatives with 2nd-order finite differences
    %
    % Maximilian Herrmann
    % Chair of Automatic Control
    % TUM School of Engineering and Design
    % Technical University of Munich
    arguments
        % Matrix of function values with size (n,N), where N are the
        % number of time nodes (i.e., time steps in the columns)
        x (:,:)

        % Time step
        h (1,1)
    end

    % First step: Forward difference quotient
    x_dot_1  = (-3*x(:,1) + 4*x(:,2) - x(:,3)) / (2*h);
    x_ddot_1 = (2*x(:,1) - 5*x(:,2) + 4*x(:,3) - x(:,4)) / (h^2);

    % Interior steps: Centered difference quotient
    x_dot_int  = (x(:,3:end) - x(:,1:end-2)) / (2*h);
    x_ddot_int = (x(:,3:end) -2*x(:,2:end-1) + x(:,1:end-2)) / (h^2);

    % Last step: Backward difference quotient
    x_dot_N  = (3*x(:,end) - 4*x(:,end-1) + x(:,end-2)) / (2*h);
    x_ddot_N = (2*x(:,end) - 5*x(:,end-1) + 4*x(:,end-2) - x(:,end-3)) / (h^2);

    x_dot  = [x_dot_1,  x_dot_int,  x_dot_N];
    x_ddot = [x_ddot_1, x_ddot_int, x_ddot_N];

end