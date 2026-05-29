function [x_dot, x_ddot] = diff4thOrder(x, h)
    %% Compute first and second derivatives with 4th-order finite differences
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

    % 5-Point FD Coefficients have been computed with: 
    % https://web.media.mit.edu/~crtaylor/calculator.html
    % See also
    % https://en.wikipedia.org/wiki/Finite_difference_coefficient

    N = size(x, 2);

    % Step 1: Forward difference quotient
    x_dot_1  = getFivePointFD(x, 1, 0:4, [-25, 48, -36, 16, -3], 12*h);
    x_ddot_1 = getFivePointFD(x, 1, 0:4, [35, -104, 114, -56, 11], 12*h^2);
    
    % Step 2: Skewed quotient
    x_dot_2  = getFivePointFD(x, 2, -1:3, [-3, -10, 18, -6, 1], 12*h);
    x_ddot_2 = getFivePointFD(x, 2, -1:3, [11, -20, 6, 4, -1], 12*h^2);

    % Interior steps 3...N-2: Centered difference quotient
    x_dot_int  = (+1*x(:,1:end-4) -  8*x(:,2:end-3) +  0*x(:,3:end-2) +  8*x(:,4:end-1) - 1*x(:,5:end-0))/(12*h);
    x_ddot_int = (-1*x(:,1:end-4) + 16*x(:,2:end-3) - 30*x(:,3:end-2) + 16*x(:,4:end-1) - 1*x(:,5:end-0))/(12*h^2);
    
    % Step N-1: Skewed quotient
    x_dot_N0  = getFivePointFD(x, N-1, -3:1, [-1, 6, -18, 10, 3], 12*h);
    x_ddot_N0 = getFivePointFD(x, N-1, -3:1, [-1, 4, 6, -20, 11], 12*h^2);

    % Last step: Backward difference quotient
    x_dot_N  = getFivePointFD(x, N, -4:0, [3, -16, 36, -48, 25], 12*h);
    x_ddot_N = getFivePointFD(x, N, -4:0, [11, -56, 114, -104, 35], 12*h^2);

    x_dot  = [x_dot_1,  x_dot_2,  x_dot_int,  x_dot_N0,  x_dot_N];
    x_ddot = [x_ddot_1, x_ddot_2, x_ddot_int, x_ddot_N0, x_ddot_N];

end

function x_dot = getFivePointFD(x, k0, kVec, cVec, cDenom)
    %% Get five-point finite difference quotient value
    arguments
        x       (:,:)   % Matrix of function values
        k0      (1,1)   % Index of the stencil "center"
        kVec    (:,1)   % Vector of (relative) sample point indices
        cVec    (:,1)   % Coefficients for sampled points
        cDenom  (:,1)   % Denominator factor (e.g., 12*h^2)
    end
    x_dot = (...
        cVec(1)*x(:,k0 + kVec(1)) ...
        + cVec(2)*x(:,k0 + kVec(2)) ...
        + cVec(3)*x(:,k0 + kVec(3)) ...
        + cVec(4)*x(:,k0 + kVec(4)) ...
        + cVec(5)*x(:,k0 + kVec(5)) ) / cDenom;
end