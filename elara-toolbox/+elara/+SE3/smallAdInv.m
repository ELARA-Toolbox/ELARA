function xi = smallAdInv(Z)
    % extract se3 element from small ad representation
    % after [Sel05, p. 68]
    % convention for se3 elements in vector form: [omega; v]
    % (see p. 56 in [Sel05]

    % Output: se3 element in vector form!

    arguments
        Z (6,6)
    end

    xi = [
        elara.SO3.skewInv(Z(1:3, 1:3));
        elara.SO3.skewInv(Z(4:end, 1:3));
        ];
end
