function gInv = SE3MatrixInv(R, x)
    % Get the (homogeneous) matrix representation of the *inverse* SE3
    % element from rotation matrix and position vector
    % See e.g. [MLS94, p.37] and lots of other sources

    arguments
        R (3,3) % Rotation matrix (element of SO(3))
        x (3,1) % Position vector (element of R^3)
    end

    gInv = [
        R.', -R.' * x
        zeros(1,3), 1
        ];
end

