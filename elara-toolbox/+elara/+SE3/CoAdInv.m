function CoAdInv = CoAdInv(g)
    % Large inverse co-adjoint representation of an element of SE(3)
    % Follows the convention for se(3) elements in vector form: [omega; v].
    %
    % This inverse can be derived easily using the matrix representation of
    % an inverse SE(3) element and the identity (Rp)^ = R(p^)R^T from
    % [LLM18, p.10].
    %
    % Input g: SE(3) element in (4x4) matrix representation
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

    % Inverse co-adjoint representation
    CoAdInv = [
        R,        elara.SO3.skew(p)*R;
        zeros(3), R
        ];
end
