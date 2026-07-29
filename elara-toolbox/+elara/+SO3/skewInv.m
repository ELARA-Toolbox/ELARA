function x = skewInv(X)
    %% Inverse hat map for so(3)
    arguments
        X (3,3)
    end

    x = [X(3,2); X(1,3); X(2,1)];
end
