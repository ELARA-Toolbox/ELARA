function CoAdInv = ColAdSE3Inv(g)
    % Large inverse Co-Ad representation (6x6 matrix) of an element of SE3
    % Follows convention for se3 elements in vector form: [omega; v].
    %
    % This inverse can be derived easily using the matrix representation of
    % an inverse SE3 element and the identity (Rp)^ = R(p^)R^T from
    % [LLM18, p.10].
    %
    % Input g: SE3 element in (4x4) matrix representation
    %
    % Maximilian Herrmann
    % Chair of Automatic Control
    % TUM School of Engineering and Design
    % Technical University of Munich

    arguments
        g (4,4)
    end

    R = g(1:3, 1:3);
    p = g(1:3, 4);

    % Inverse Co-Ad representation
    CoAdInv = [
        R,        skew(p)*R;
        zeros(3), R
        ];
end
