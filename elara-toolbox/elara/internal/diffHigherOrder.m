function [x_dot, x_ddot] = diffHigherOrder(x, h, order)
    %% Compute first and second derivatives with Higher-Order finite differences
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

        % Finite difference order
        order (1,1) double {mustBeMember(order, [1,2,4])} = 2;
    end

    switch order
        case 1
            [x_dot, x_ddot] = diff1stOrder(x, h);
        case 2
            [x_dot, x_ddot] = diff2ndOrder(x, h);
        case 4
            [x_dot, x_ddot] = diff4thOrder(x, h);
        otherwise
            error("Finite difference order not defined.");
    end
end

function [x_dot, x_ddot] = diff1stOrder(x, h)
    %% Compute derivatives with 1st-order finite differences
    arguments
        % Matrix of function values with size (n,N), where N are the
        % number of time nodes (i.e., time steps in the columns)
        x (:,:)

        % Time step
        h (1,1)
    end

    % First step: Forward difference quotient
    x_dot_1  = (x(:,2) - x(:,1)) / h;
    x_ddot_1 = (x(:,3) - 2*x(:,2) + x(:,1)) / h^2;

    % Interior steps
    x_dot_int  = (x(:,2:end-1) - x(:,1:end-2)) / h;
    x_ddot_int = (x(:,3:end) - 2*x(:,2:end-1) + x(:,1:end-2)) / h^2;

    % Last step: Backward difference quotient
    x_dot_N  = (x(:,end) - x(:,end-1)) / h;
    x_ddot_N = (x(:,end) - 2*x(:,end-1) + x(:,end-2)) / h^2;

    x_dot  = [x_dot_1,  x_dot_int,  x_dot_N];
    x_ddot = [x_ddot_1, x_ddot_int, x_ddot_N];
end