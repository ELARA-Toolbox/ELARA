function Z = CoSadSE3(xi)
    % small Co-ad representation (6x6 matrix) of an element of se3
    % after [Sel05, p. 68]
    % convention for se3 elements in vector form: [omega; v] (also [Lee08] convention)
    % (see p. 56 in [Sel05])
    %
    % Input xi: se3 element in vector form
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
        -skew(omega), -skew(v);
        zeros(3),     -skew(omega);
        ];
end