function g = SE3Matrix(R, x)
    % Get the (homogeneous) matrix representation of an element of SE(3)
    % from rotation matrix and position vector
    % See e.g. [LLM18, p.314], [MLS94] and lots of other sources
    %
    % Also works with an array of R/x elements with dimension (3,3,N)
    % and (3,N), where N is the number of elements.

    arguments
        R (3,3,:,:) % Rotation matrix (element of SO(3))
        x (3,:,:)   % Position vector (element of R^3)
    end

    % ToDo: assert that nr. of elements in R and x is equal?

    g = zeros(4,4,size(R,3),size(R,4));
    g(1:3,1:3,:,:) = R;
    g(1:3,4,:,:)   = x;
    g(4,4,:,:)     = 1;
end

