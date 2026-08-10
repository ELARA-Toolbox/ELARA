function [B, B_d1t, B_d2t, tau] = computeBSplineBasis(nCP, p, tVec, t0, tEnd)
    %% Compute B-spline basis matrices at specified times
    % and the basis matrices for the first and second time derivatives.
    % With the basis matrix, the B-spline path can be computed with
    %      x = B * z,
    % where z contains the control points,
    % and the derivatives with
    %   x_dt = B_d1t * z,
    %  x_ddt = B_d2t * z.
    %
    % See e.g., Les Piegl, Wayne Tiller: The NURBS Book, chapter 2.1.
    %
    % Maximilian Herrmann
    % Chair of Automatic Control
    % TUM School of Engineering and Design
    % Technical University of Munich
    arguments
        % Number of control points
        nCP     (1,1) double

        % Spline degree
        p       (1,1) double

        % Vector of evaluation points
        tVec    (:,1) double

        % Beginning and end of the domain of t
        % (required for the case where the evaluation vector tVec does not
        % start and/or end at t0/tEnd)
        t0      (1,1) double
        tEnd    (1,1) double
    end

    % Number of evaluation points
    S = length(tVec);

    % Compute spline parameters
    K = nCP - 1;
    m = K + p + 1;

    % Define knot vector
    % (p+1)-fold initial and final knot
    nT = m + 1;
    T = [
        ones(1,p)*t0, ...
        linspace(t0, tEnd, nT-(2*p)), ...
        ones(1,p)*tEnd
        ];


    %% Construct basis functions
    % using the Cox-de Boor recursion formulas

    % Zero-degree basis functions
    B0 = zeros(S, m);
    for i = 1:m
        for k = 1:S

            % Adjusted to get non-zero basis function at last time step
            % (with multiple end knots)
            B0(k,i) = (tVec(k) >= T(i) && tVec(k) < T(i+1)) || ...
                (tVec(k) == T(end) && T(i+1) == T(end));
        end
    end

    % Higher-order basis functions
    B = B0;
    B_d1t = B0;

    for pN = 1:p
        Bp0 = B;
        B = zeros(S, m-pN);
        if nargout > 1
            B_d1t_p0 = B_d1t;
            B_d1t = zeros(S, m-pN);
            B_d2t = zeros(S, m-pN);
        end
        for i = 1:(m-pN)
            for k = 1:S
                f1 = ( tVec(k) - T(i) )/( T(i+pN)-T(i) );
                if(~isfinite(f1))
                    f1 = 0;
                end
                f2 = ( T(i+pN+1) - tVec(k) ) / ( T(i+pN+1) - T(i+1) );
                if(~isfinite(f2))
                    f2 = 0;
                end
                B(k,i) =  f1 * Bp0(k, i) + f2 * Bp0(k, i+1);

                if nargout > 1
                    % First derivative
                    f1_dt = pN /( T(i+pN)-T(i) );
                    if(~isfinite(f1_dt))
                        f1_dt = 0;
                    end
                    f2_dt = pN / ( T(i+pN+1) - T(i+1) );
                    if(~isfinite(f2_dt))
                        f2_dt = 0;
                    end
                    B_d1t(k,i) =  f1_dt * Bp0(k, i) - f2_dt * Bp0(k, i+1);

                    % Second derivative
                    if nargout > 2
                        B_d2t(k,i) =  f1_dt * B_d1t_p0(k, i) - f2_dt * B_d1t_p0(k, i+1);
                    end
                end
            end
        end
    end

    %% Additionally: Compute Greville abscissae
    % Used e.g., to visualize distribution of control points over time
    % See e.g.:
    %  https://web.mit.edu/hyperbook/Patrikalakis-Maekawa-Cho/node16.html
    %  https://www.gnu.org/software/gsl/doc/html/bspline.html#greville-abscissae
    if nargout > 3
        tau = zeros(nCP,1);
        for i = 1:nCP
            tau(i) = mean(T(i+1:i+p));
        end
    end
end

