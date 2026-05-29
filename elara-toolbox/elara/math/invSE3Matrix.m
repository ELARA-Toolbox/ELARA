function gInv = invSE3Matrix(g)
    % Invert the (homogeneous) matrix representation of an SE(3) element
    % (containing a rotation matrix and a position vector)
    % See e.g. [MLS94, p.37] and lots of other sources

    arguments
        g (4,4) % SE(3) Matrix
    end

    R = g(1:3, 1:3);
    gInv = [
        R.', -R.' * g(1:3, 4)
        zeros(1,3), 1
        ];
end

