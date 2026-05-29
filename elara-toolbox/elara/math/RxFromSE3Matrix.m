function [R, x] = RxFromSE3Matrix( g )
    % Get the rotation matrix and position vector from the
    % (homogeneous) matrix representation of an element of SE(3)
    % See e.g. [LLM18, p.314], [MLS94] and lots of other sources
    %
    % Also works with an array of SE3 matrices with dimension (4,4,N),
    % where N is the number of elements.
    % The outputs are then:
    %   R   Array of rotation matrices of dimension (3,3,N)
    %   x   Array of position vectors of dimension (3,N)

    arguments
        g (4,4,:,:)
    end

    R = g(1:3, 1:3, :, :);
    x = reshape( g(1:3, 4, :), [3, size(g, 3),size(g, 4)]);
end

