function Z = smallAd(xi)
    % Small adjoint representation of an element of se(3)
    % after [Sel05, p. 68]
    % Convention for se(3) elements in vector form: [omega; v] (also [Lee08])
    % (see p. 56 in [Sel05])
    %
    % Input xi: se(3) element in vector form
    %
    % Maximilian Herrmann
    % Chair of Automatic Control
    % TUM School of Engineering and Design
    % Technical University of Munich

    arguments
        xi (6,1)
    end

    omega = xi(1:3);
    v     = xi(4:end);

    Z = [
        elara.SO3.skew(omega), zeros(3);
        elara.SO3.skew(v),     elara.SO3.skew(omega);
        ];
end
