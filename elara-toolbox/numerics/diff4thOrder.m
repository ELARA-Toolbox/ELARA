function [x_dot, x_ddot] = diff4thOrder(x, h)
    %% Compute first and second derivatives with 4th-order finite differences
    %
    % Maximilian Herrmann
    % Chair of Automatic Control
    % TUM School of Engineering and Design
    % Technical University of Munich
    arguments
        % Matrix of function values with size (n,N), where N >= 6 is the
        % number of time nodes
        x (:,:)

        % Time step
        h (1,1)
    end

    % Finite-difference coefficients can be generated with the algorithm in
    % B. Fornberg, Mathematics of Computation 51 (1988), pp. 699-706:
    % https://doi.org/10.1090/S0025-5718-1988-0935077-0

    N = size(x, 2);

    % Step 1: Forward difference quotient
    x_dot_1  = getFivePointFD(x, 1, 0:4, [-25, 48, -36, 16, -3], 12*h);
    x_ddot_1 = getSixPointFD(x, 1, 0:5, [45, -154, 214, -156, 61, -10], 12*h^2);
    
    % Step 2: Skewed quotient
    x_dot_2  = getFivePointFD(x, 2, -1:3, [-3, -10, 18, -6, 1], 12*h);
    x_ddot_2 = getSixPointFD(x, 2, -1:4, [10, -15, -4, 14, -6, 1], 12*h^2);

    % Interior steps 3...N-2: Centered difference quotient
    x_dot_int  = (+1*x(:,1:end-4) -  8*x(:,2:end-3) +  0*x(:,3:end-2) +  8*x(:,4:end-1) - 1*x(:,5:end-0))/(12*h);
    x_ddot_int = (-1*x(:,1:end-4) + 16*x(:,2:end-3) - 30*x(:,3:end-2) + 16*x(:,4:end-1) - 1*x(:,5:end-0))/(12*h^2);
    
    % Step N-1: Skewed quotient
    x_dot_N0  = getFivePointFD(x, N-1, -3:1, [-1, 6, -18, 10, 3], 12*h);
    x_ddot_N0 = getSixPointFD(x, N-1, -4:1, [1, -6, 14, -4, -15, 10], 12*h^2);

    % Last step: Backward difference quotient
    x_dot_N  = getFivePointFD(x, N, -4:0, [3, -16, 36, -48, 25], 12*h);
    x_ddot_N = getSixPointFD(x, N, -5:0, [-10, 61, -156, 214, -154, 45], 12*h^2);

    x_dot  = [x_dot_1,  x_dot_2,  x_dot_int,  x_dot_N0,  x_dot_N];
    x_ddot = [x_ddot_1, x_ddot_2, x_ddot_int, x_ddot_N0, x_ddot_N];

end

function x_dot = getSixPointFD(x, k0, kVec, cVec, cDenom)
    %% Get six-point finite difference quotient value
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
        + cVec(5)*x(:,k0 + kVec(5)) ...
        + cVec(6)*x(:,k0 + kVec(6)) ) / cDenom;
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
